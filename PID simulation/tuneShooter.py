"""
tuneShooter.py — PIDF tuning for the Motor-Rack-Spring system
Pololu 37D 70:1 10V, rack-and-pinion (r=35.2/2 mm), spring k=100 N/m

Defines the plant, cost weights, and starting points, then delegates
optimisation to pidf_optimizer.run_multistart_optimization().
"""

import numpy as np
import matplotlib.pyplot as plt

from pidf_optimizer import simulate_euler, get_metrics, cost_fn, run_multistart_optimization


# ── 1. SYSTEM PARAMETERS ───────────────────────────────────────────────────
#
# Kt and Kb are identified from a no-load step characterisation:
#   omega_out / V = 1.47 / (0.5451 s + 1)
#
# For a DC motor + N:1 gearbox with b ≈ 0 (back-EMF dominates friction):
#   K_DC  = 1 / (N · Kb)           →  Kb = 1 / (N · K_DC)
#   T_m   = Jm_eff · R / (Kt · Kb) →  Jm_eff = T_m · Kt · Kb / R
#
# Extracted values:
#   Kb   = 1 / (70 × 1.47)                          = 9.73e-3 V·s/rad
#   Kt   ≈ Kb (SI units, ideal motor)               = 9.73e-3 N·m/A
#   Jm_eff = 0.5451 × (9.73e-3)² / 2.31            = 2.23e-5 kg·m²
#            (motor rotor + full gearbox inertia at motor shaft)
#
K_DC_NOLOAD = 2.47    # Measured no-load DC gain  [rad_out/s / V]
T_M_NOLOAD  = 0.2451  # Measured mechanical time constant [s]

R = 2.31       # Armature resistance [ohm]
N  = 70        # Gear ratio
Kb = 1.0 / (N * K_DC_NOLOAD)              # 9.73e-3 V·s/rad
Kt = Kb                                    # equal for SI (ideal motor)
Jm = T_M_NOLOAD * Kt * Kb / R             # 2.23e-5 kg·m²  (motor + gearbox)
b  = 0.0       # viscous damping ≈ 0 (back-EMF term dominates; confirmed by TF fit)
r = 0.0352/2     # Pinion pitch radius [m]
M = 0.085      # Load + rack mass [kg]
k = 60        # Spring constant [N/m]
F_PRE     = .5    # Spring preload force [N]  (constant force opposing +x displacement)
F_COULOMB = 0.4    # Coulomb (static/kinetic) friction force on rack [N]
V_MAX = 10.0   # Supply voltage limit [V]
MOTOR_SIGNAL_MAX = 400
V_TO_SIGNAL = MOTOR_SIGNAL_MAX/V_MAX

# Lumped coefficients
Nr = N / r
M_eff = M + Jm * Nr**2
b_eff = b * Nr**2
alpha = (Kt * N) / (r * R * M_eff)
beta  = b_eff / M_eff + (Kt * Kb * Nr**2) / (R * M_eff)
gamma = k / M_eff

# Disturbance accelerations (divided by M_eff for the integrator)
A_PRE     = F_PRE     / M_eff   # [m/s²]  constant preload disturbance
A_COULOMB = F_COULOMB / M_eff   # [m/s²]  Coulomb friction magnitude

print("=== Plant ===")
print(f"  M_eff = {M_eff:.4f} kg  ({100 * Jm * Nr**2 / M_eff:.1f}% reflected inertia)")
print(f"  alpha = {alpha:.5f} | beta = {beta:.5f} | gamma = {gamma:.5f}")
print(f"  Open-loop: wn = {np.sqrt(gamma):.3f} rad/s, zeta = {beta / (2 * np.sqrt(gamma)):.3f}")
print(f"  Disturbances: a_pre = {A_PRE:.4f} m/s²  (F_pre={F_PRE} N),"
      f"  a_coulomb = {A_COULOMB:.4f} m/s²  (F_c={F_COULOMB} N)")


# ── 2. OPTIMISATION SETUP ──────────────────────────────────────────────────

STEPS   = [0.005, 0.025, 0.050]   # 5 mm, 25 mm, 50 mm
W_RISE      = 4.0
W_OVER      = 3.0
W_SETTLE    = 2.0
W_EFFORT    = 1.0
DT_SIM      = 0.004;    
T_END       = 3.0;     

bounds = [
    (0.0,  5000.0),   # Kp
    (0.0, 30000.0),   # Ki
    (0.0,   300.0),   # Kd
    (0.0,  3000.0),   # a
]

# Starting points: taken from robot
# p0  = 3 * np.sqrt(gamma)
Kp0 = 6000 / V_TO_SIGNAL
Ki0 = 0 / V_TO_SIGNAL
Kd0 = 1000 / V_TO_SIGNAL
a = 600 / V_TO_SIGNAL

starts = np.array([
    [Kp0,    Ki0,   Kd0,   a],     # Pole-placement baseline
    [1549.0, 9.6,   31.3,  0.0],     # Pre-validated from Python
    [1596.9, 1.8,   32.5,  7.6],     # Python optimised (best known)
    [1200.0, 30.0,  25.0,  0.0],
    [900.0,  5.0,   45.0,  0.0],
    [2000.0, 100.0, 50.0,  100.0],
])


# ── 3. RUN OPTIMISATION ────────────────────────────────────────────────────

best_gains, best_j, all_results = run_multistart_optimization(
    starts=starts,
    bounds=bounds,
    steps=STEPS,
    alpha=alpha, beta=beta, gamma=gamma, v_max=V_MAX,
    w_rise=W_RISE, w_over=W_OVER, w_settle=W_SETTLE, w_effort=W_EFFORT,
    dt_sim=DT_SIM, t_end=T_END,    a_pre=A_PRE, a_coulomb=A_COULOMB,)

Kp, Ki, Kd, a = best_gains

print("\n=== Optimised Gains ===")
print(f"  Kp = {Kp:.4f}")
print(f"  Ki = {Ki:.4f}")
print(f"  Kd = {Kd:.4f}")
print(f"  a  = {a:.4f}")
print(f"  Cost J = {best_j:.5f}")

print("\n=== Optimised Gains In motor signal units ===")
print(f"  Kp = {Kp * 400/10:.4f}")
print(f"  Ki = {Ki * 400/10:.4f}")
print(f"  Kd = {Kd * 400/10:.4f}")
print(f"  a  = {a * 400/10:.4f}")
print(f"  Cost J = {best_j:.5f}")


# ── 4. COMPARE: INITIAL vs OPTIMISED ──────────────────────────────────────

gains_init = np.array([Kp0, Ki0, Kd0, 0.0])
gains_opt = best_gains

print("\n=== Step Response Metrics ===")
print(f"{'Config':<12}  {'Step':<6}  {'Rise(s)':<8}  {'OS(%)':<8}  {'Settle(s)':<10}  {'Vpeak(V)':<10}")
print("-" * 65)

for label, g in [("Initial", gains_init), ("Optimised", gains_opt)]:
    for xref in STEPS:
        t, x, v = simulate_euler(g, xref, alpha, beta, gamma, V_MAX, DT_SIM, T_END,
                                   a_pre=A_PRE, a_coulomb=A_COULOMB)
        rise, over, settle = get_metrics(t, x, xref)
        print(
            f"{label:<12}  {xref * 1000:4.0f}mm  "
            f"{rise:8.3f}  {over * 100:7.2f}%  {settle:10.3f}  {np.max(np.abs(v)):10.2f}"
        )


# ── 5. PLOTS ───────────────────────────────────────────────────────────────

step_labels = ["5mm", "25mm", "50mm"]

# Plot 1: Step response comparison
plt.figure("Step Response: Initial vs Optimised", figsize=(9, 5.2))
for s, xref in enumerate(STEPS):
    t, x_i, _ = simulate_euler(gains_init, xref, alpha, beta, gamma, V_MAX, DT_SIM, T_END,
                                a_pre=A_PRE, a_coulomb=A_COULOMB)
    _, x_o, _ = simulate_euler(gains_opt, xref, alpha, beta, gamma, V_MAX, DT_SIM, T_END,
                                a_pre=A_PRE, a_coulomb=A_COULOMB)

    plt.plot(t, x_i * 1000, "--", linewidth=1.5, label=f"Initial - {step_labels[s]}")
    plt.plot(t, x_o * 1000, "-", linewidth=2.2, label=f"Optimised - {step_labels[s]}")
    plt.axhline(xref * 1000, linestyle=":", linewidth=0.8, color="gray")

plt.xlabel("Time (s)")
plt.ylabel("Position (mm)")
plt.title("PIDF Step Response - Initial (dashed) vs Gradient-Descent Optimised (solid)")
plt.legend(loc="lower right", ncol=2)
plt.grid(True)
plt.xlim(0, T_END)
plt.ylim(-3, 60)

# Plot 2: Control voltage (optimised, 50 mm step)
plt.figure("Control Voltage", figsize=(9, 3.8))
t, x_o50, v_o50 = simulate_euler(gains_opt, 0.050, alpha, beta, gamma, V_MAX, DT_SIM, T_END,
                                  a_pre=A_PRE, a_coulomb=A_COULOMB)

ax1 = plt.gca()
ax1.plot(t, x_o50 * 1000, linewidth=2, label="x(t)")
ax1.axhline(50, linestyle=":", linewidth=1, label="x_ref")
ax1.set_ylabel("Position (mm)")
ax1.set_ylim(-2, 58)
ax1.set_xlabel("Time (s)")
ax1.set_title("Optimised PIDF - 50mm Step: Position and Control Voltage")
ax1.grid(True)

ax2 = ax1.twinx()
ax2.plot(t, v_o50, linewidth=1.5, label="V(t)")
ax2.axhline(V_MAX, linestyle=":", linewidth=1, label="+V_max")
ax2.axhline(-V_MAX, linestyle=":", linewidth=1, label="-V_max")
ax2.set_ylabel("Voltage V(t) [V]")
ax2.set_ylim(-14, 14)

lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc="center right")

# Plot 3: Cost landscape — Kp vs Kd slice (Ki, a fixed at optimum)
plt.figure("Cost Landscape", figsize=(7.8, 5.4))
Kp_range = np.linspace(10, 5000, 60)
Kd_range = np.linspace(0, 1000, 60)
KP_g, KD_g = np.meshgrid(Kp_range, Kd_range)
J_map = np.zeros_like(KP_g)

print("Computing cost landscape (60x60 grid)...")
for ii in range(KP_g.shape[0]):
    for jj in range(KP_g.shape[1]):
        g = [KP_g[ii, jj], Ki, KD_g[ii, jj], a]
        J_map[ii, jj] = cost_fn(
            g, STEPS, alpha, beta, gamma, V_MAX,
            W_RISE, W_OVER, W_SETTLE, W_EFFORT, DT_SIM, T_END,
            a_pre=A_PRE, a_coulomb=A_COULOMB,
        )

# Avoid issues with log color scale if any values are exactly zero or negative
J_map_safe = np.maximum(J_map, 1e-9)

contour = plt.contourf(KP_g, KD_g, J_map_safe, levels=25)
plt.colorbar(contour)
plt.plot(Kp, Kd, marker="*", markersize=14, linewidth=0, label="Optimum")
plt.plot(Kp0, Kd0, marker="x", markersize=12, linewidth=0, label="Initial")
plt.xlabel("K_p")
plt.ylabel("K_d")
plt.title(f"Cost landscape (K_i={Ki:.2f}, a={a:.2f} fixed at optimum)")
plt.legend(loc="upper right")
plt.yscale("linear")
plt.xscale("linear")
plt.gca().set_facecolor("white")

# Log color scale approximation
try:
    contour.set_norm(plt.matplotlib.colors.LogNorm(vmin=J_map_safe.min(), vmax=J_map_safe.max()))
    plt.clf()
    plt.figure("Cost Landscape", figsize=(7.8, 5.4))
    contour = plt.contourf(
        KP_g, KD_g, J_map_safe, levels=25,
        norm=plt.matplotlib.colors.LogNorm(vmin=J_map_safe.min(), vmax=J_map_safe.max())
    )
    plt.colorbar(contour)
    plt.plot(Kp, Kd, marker="*", markersize=14, linewidth=0, label="Optimum")
    plt.plot(Kp0, Kd0, marker="x", markersize=12, linewidth=0, label="Initial")
    plt.xlabel("K_p")
    plt.ylabel("K_d")
    plt.title(f"Cost landscape (K_i={Ki:.2f}, a={a:.2f} fixed at optimum)")
    plt.legend(loc="upper right")
except Exception:
    pass

# Plot 4: Spring constant robustness
plt.figure("Robustness to k", figsize=(7.8, 3.8))
k_vals = [50, 100, 200, 400]
xref_test = 0.025  # 25 mm step

for kk in k_vals:
    gamma_k = kk / M_eff
    alpha_k = alpha
    beta_k = beta
    t, x_k, _ = simulate_euler(gains_opt, xref_test, alpha_k, beta_k, gamma_k, V_MAX, DT_SIM, T_END,
                                a_pre=A_PRE, a_coulomb=A_COULOMB)
    plt.plot(t, x_k * 1000, linewidth=2, label=f"k = {kk} N/m")

plt.axhline(xref_test * 1000, linestyle=":", linewidth=1.2, label="Reference")
plt.xlabel("Time (s)")
plt.ylabel("Position (mm)")
plt.title("Robustness: Optimised Gains Applied Across Spring Constants (25mm step)")
plt.legend(loc="lower right")
plt.grid(True)
plt.xlim(0, T_END)

print("\nDone - 4 figures generated.")
plt.show()