"""
tuneDirectDrive.py — Example: PIDF tuning for a direct-drive motor with a point-mass arm
=========================================================================================

This script is intentionally simple to demonstrate how to use pidf_optimizer
with the transfer-function (num/den) interface.

Physical setup
--------------
    A brushed DC motor drives a rigid arm of length r_arm [m] with a point
    mass m_load [kg] at its tip.  There is no gearbox.  The output is the
    angular position θ [rad] of the arm.

Equation of motion
------------------
    J_total · θ'' = Kt · I  −  b · θ'
    V  = R · I + Kb · θ'

    Substituting I = (V − Kb·θ') / R:
        θ'' = (Kt / (R·J_total)) · V  −  (b/J_total + Kt·Kb/(R·J_total)) · θ'
            =         alpha · V        −                 beta · θ'

    Transfer function (V → θ):
        θ(s)/V(s) = alpha / (s² + beta·s)
                  = alpha / (s · (s + beta))

    In MATLAB tf() notation:  num = [alpha],  den = [1, beta, 0]
    The trailing zero in den is the integrator pole (position from velocity).

Note on gravity
---------------
    Gravity creates a position-dependent torque τ_g = m·g·r·cos(θ).
    At small angles this is approximately a negative spring (destabilising).
    For large-angle control, a more detailed simulation is needed.
    This example assumes the arm moves near-horizontal or that gravity
    compensation is handled separately (e.g., via the feed-forward gain a).
"""

import numpy as np
import matplotlib.pyplot as plt

from pidf_optimizer import (
    tf_to_ss, simulate_ss, get_metrics,
    run_multistart_optimization,
)


# ── 1. MOTOR & LOAD PARAMETERS ─────────────────────────────────────────────

R     = 3.0      # Armature resistance [Ω]
Kt    = 0.05     # Torque constant  [N·m/A]
Kb    = 0.05     # Back-EMF constant [V·s/rad]  (= Kt in SI for ideal motor)
Jm    = 1e-5     # Motor rotor inertia [kg·m²]
b     = 1e-4     # Motor viscous damping [N·m·s/rad]
V_MAX = 12.0     # Supply voltage limit [V]
MOTOR_SIGNAL_MAX = 400
V_TO_SIGNAL = MOTOR_SIGNAL_MAX / V_MAX

# Load: point mass on a rigid arm
m_load = 0.10    # Point mass [kg]
r_arm  = 0.15    # Arm length (pivot to mass) [m]
J_load = m_load * r_arm**2   # Point-mass inertia about pivot [kg·m²]

J_total = Jm + J_load        # Total inertia at motor shaft [kg·m²]


# ── 2. DERIVE TRANSFER FUNCTION ─────────────────────────────────────────────

alpha = Kt / (R * J_total)                         # [rad/s² per V]
beta  = b / J_total + (Kt * Kb) / (R * J_total)   # [rad/s per rad/s] = [1/s]

#  θ(s)/V(s) = alpha / (s² + beta·s)
#  num = [alpha],  den = [1, beta, 0]
num = [alpha]
den = [1.0, beta, 0.0]

print("=== Direct-Drive Motor + Arm ===")
print(f"  J_load  = {J_load*1e6:.2f} g·cm²  (m={m_load*1e3:.0f}g, r={r_arm*100:.0f}cm)")
print(f"  J_total = {J_total*1e6:.2f} g·cm²")
print(f"  alpha   = {alpha:.3f} rad/s²/V")
print(f"  beta    = {beta:.4f} s⁻¹  (open-loop velocity pole at s = -{beta:.4f})")
print(f"  TF: theta/V = {alpha:.3f} / (s² + {beta:.4f}·s)")
print(f"  No-load max speed @ V_MAX: {alpha / beta * V_MAX:.2f} rad/s  "
      f"({np.degrees(alpha / beta * V_MAX):.1f} deg/s)")

# Sanity-check: convert to state-space and confirm matrices
A_ss, B_ss, C_ss, D_ss = tf_to_ss(num, den)
print(f"\n  State-space (controllable canonical form):")
print(f"    A = {A_ss.tolist()}")
print(f"    B = {B_ss.flatten().tolist()}")
print(f"    C = {C_ss.flatten().tolist()}")


# ── 3. OPTIMISATION SETUP ──────────────────────────────────────────────────

# Reference steps in RADIANS
DEG = np.pi / 180
STEPS = [30 * DEG, 60 * DEG, 90 * DEG]   # 30°, 60°, 90°

W_RISE   = 4.0
W_OVER   = 3.0
W_SETTLE = 2.0
W_EFFORT = 1.0
DT_SIM   = 0.002    # 2 ms — shorter dt for a faster, lighter system
T_END    = 4.0

bounds = [
    (0.0,  2000.0),   # Kp
    (0.0,  5000.0),   # Ki
    (0.0,   500.0),   # Kd
    (0.0,   500.0),   # a  (feed-forward)
]

# Physics-informed starting gains for a double integrator.
# Target closed-loop bandwidth wc [rad/s]:
#   Kp ≈ wc² / alpha  (sets the proportional stiffness)
#   Kd ≈ 2·wc / alpha  (critical damping)
wc  = 5.0                    # target crossover [rad/s]
Kp0 = wc**2 / alpha
Ki0 = 0.0
Kd0 = 2.0 * wc / alpha

# Feed-forward to compensate arm gravity at the target angle (optional).
# a·x_ref produces a constant voltage offset V_ff = a·theta_ref.
# At θ_ref: τ_gravity = m·g·r·cos(θ_ref) → V_ff = τ_gravity / (Kt·N/R) = τ_gravity·R/Kt
# Simplified constant: a_ff ≈ m·g·r·R / Kt / theta_ref_typical  → rough value
g_gravity = 9.81
V_gravity_mid = m_load * g_gravity * r_arm * R / Kt   # V to hold arm horizontal
a_ff0 = V_gravity_mid / (np.mean(STEPS))               # scale to mid-step

print(f"\n  Bandwidth target: wc = {wc} rad/s")
print(f"  Starting gains: Kp={Kp0:.3f}  Ki={Ki0:.3f}  Kd={Kd0:.3f}  a={a_ff0:.3f}")

starts = np.array([
    [Kp0,      Ki0,  Kd0,      a_ff0],    # Physics-derived baseline
    [Kp0,      Ki0,  Kd0,      0.0],      # No feed-forward
    [Kp0 * 2,  Ki0,  Kd0,      a_ff0],
    [Kp0,      Ki0,  Kd0 * 2,  a_ff0],
    [Kp0 / 2,  1.0,  Kd0,      a_ff0],
    [Kp0 * 3,  2.0,  Kd0 * 3,  0.0],
])


# ── 4. RUN OPTIMISATION ────────────────────────────────────────────────────

print()
best_gains, best_j, all_results = run_multistart_optimization(
    starts=starts,
    bounds=bounds,
    steps=STEPS,
    v_max=V_MAX,
    num=num, den=den,           # ← TF interface
    w_rise=W_RISE, w_over=W_OVER, w_settle=W_SETTLE, w_effort=W_EFFORT,
    dt_sim=DT_SIM, t_end=T_END,
)

Kp, Ki, Kd, a = best_gains

print("\n=== Optimised Gains ===")
print(f"  Kp = {Kp:.4f}")
print(f"  Ki = {Ki:.4f}")
print(f"  Kd = {Kd:.4f}")
print(f"  a  = {a:.4f}  (feed-forward)")
print(f"  Cost J = {best_j:.5f}")

print("\n=== Optimised Gains in motor signal units ===")
print(f"  Kp = {Kp * V_TO_SIGNAL:.2f}")
print(f"  Ki = {Ki * V_TO_SIGNAL:.2f}")
print(f"  Kd = {Kd * V_TO_SIGNAL:.2f}")
print(f"  a  = {a  * V_TO_SIGNAL:.2f}")


# ── 5. STEP RESPONSE METRICS ───────────────────────────────────────────────

gains_init = np.array([Kp0, Ki0, Kd0, a_ff0])
gains_opt  = best_gains

print("\n=== Step Response Metrics ===")
print(f"{'Config':<12}  {'Step':>6}  {'Rise(s)':<8}  {'OS(%)':<8}  {'Settle(s)':<10}  {'Vpeak(V)':<10}")
print("-" * 68)

for label, g in [("Initial", gains_init), ("Optimised", gains_opt)]:
    for xref in STEPS:
        t, x, v = simulate_ss(g, xref, A_ss, B_ss, C_ss, D_ss, V_MAX, DT_SIM, T_END)
        rise, over, settle = get_metrics(t, x, xref)
        print(
            f"{label:<12}  {np.degrees(xref):5.0f}°  "
            f"{rise:8.3f}  {over * 100:7.2f}%  {settle:10.3f}  {np.max(np.abs(v)):10.2f}"
        )


# ── 6. PLOTS ───────────────────────────────────────────────────────────────

step_labels = [f"{int(np.degrees(s))}°" for s in STEPS]

# Plot 1: Step response — initial vs optimised
plt.figure("Step Response", figsize=(9, 5.2))
for s, xref in enumerate(STEPS):
    t, x_i, _ = simulate_ss(gains_init, xref, A_ss, B_ss, C_ss, D_ss, V_MAX, DT_SIM, T_END)
    _, x_o, _ = simulate_ss(gains_opt,  xref, A_ss, B_ss, C_ss, D_ss, V_MAX, DT_SIM, T_END)

    plt.plot(t, np.degrees(x_i), "--", linewidth=1.5, label=f"Initial - {step_labels[s]}")
    plt.plot(t, np.degrees(x_o), "-",  linewidth=2.2, label=f"Optimised - {step_labels[s]}")
    plt.axhline(np.degrees(xref), linestyle=":", linewidth=0.8, color="gray")

plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title("Direct-Drive PIDF Step Response — Initial (dashed) vs Optimised (solid)")
plt.legend(loc="lower right", ncol=2)
plt.grid(True)
plt.xlim(0, T_END)

# Plot 2: Control voltage for the largest step
plt.figure("Control Voltage", figsize=(9, 3.8))
xref_large = STEPS[-1]
t, x_oL, v_oL = simulate_ss(gains_opt, xref_large, A_ss, B_ss, C_ss, D_ss, V_MAX, DT_SIM, T_END)

ax1 = plt.gca()
ax1.plot(t, np.degrees(x_oL), linewidth=2, label="θ(t)")
ax1.axhline(np.degrees(xref_large), linestyle=":", linewidth=1, label="θ_ref")
ax1.set_ylabel("Angle (deg)")
ax1.set_xlabel("Time (s)")
ax1.set_title(f"Optimised PIDF — {step_labels[-1]} Step: Angle and Control Voltage")
ax1.grid(True)

ax2 = ax1.twinx()
ax2.plot(t, v_oL, linewidth=1.5, color="tab:orange", label="V(t)")
ax2.axhline( V_MAX, linestyle=":", linewidth=1, color="tab:orange", label="+V_max")
ax2.axhline(-V_MAX, linestyle=":", linewidth=1, color="tab:orange", label="-V_max")
ax2.set_ylabel("Voltage V(t) [V]")
ax2.set_ylim(-V_MAX * 1.3, V_MAX * 1.3)

lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc="center right")

# Plot 3: Robustness to tip mass variation
plt.figure("Robustness to Tip Mass", figsize=(7.8, 3.8))
mass_vals  = [0.05, 0.10, 0.20, 0.40]    # kg
xref_test  = STEPS[1]                     # 60° step

for mm in mass_vals:
    J_t   = Jm + mm * r_arm**2
    alpha_k = Kt / (R * J_t)
    beta_k  = b / J_t + (Kt * Kb) / (R * J_t)
    A_k, B_k, C_k, D_k = tf_to_ss([alpha_k], [1.0, beta_k, 0.0])
    t, x_k, _ = simulate_ss(gains_opt, xref_test, A_k, B_k, C_k, D_k, V_MAX, DT_SIM, T_END)
    plt.plot(t, np.degrees(x_k), linewidth=2, label=f"m = {mm*1e3:.0f} g")

plt.axhline(np.degrees(xref_test), linestyle=":", linewidth=1.2, label="Reference")
plt.xlabel("Time (s)")
plt.ylabel("Angle (deg)")
plt.title(f"Robustness: Optimised Gains Across Tip Masses ({step_labels[1]} step)")
plt.legend(loc="lower right")
plt.grid(True)
plt.xlim(0, T_END)

print("\nDone — 3 figures generated.")
plt.show()
