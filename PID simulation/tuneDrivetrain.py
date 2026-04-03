"""
tuneDrivetrain.py — PIDF tuning for the robot drivetrain
Pololu motor, belt/sprocket reduction, rubber wheels

Plant (double integrator — no restoring spring):
    x'' = alpha·V − beta·x'
    alpha = Kt·N / (r·R·M_eff)
    beta  = Kt·Kb·Nr² / (R·M_eff)   [back-EMF damping]

Defines the plant, cost weights, and starting points, then delegates
optimisation to pidf_optimizer.run_multistart_optimization().
"""

import numpy as np
import matplotlib.pyplot as plt
import os

from pidf_optimizer import (
    simulate_euler, simulate_euler_ramp,
    get_metrics, get_ramp_metrics,
    cost_fn, cost_fn_ramp,
    evaluate_ramp_cost_grid,
    run_multistart_optimization,
)


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
K_DC_NOLOAD = 2.25    # Measured no-load DC gain  [rad_out/s / V]
T_M_NOLOAD  = 0.1687  # Measured mechanical time constant [s]

R = 2.31       # Armature resistance [ohm]
N  = 50 / 1.5        # Gear ratio
Kb = 1.0 / (N * K_DC_NOLOAD)              # 9.73e-3 V·s/rad
Kt = Kb                                    # equal for SI (ideal motor)
Jm = T_M_NOLOAD * Kt * Kb / R             # 2.23e-5 kg·m²  (motor + gearbox)
b  = 0.0        # viscous damping ≈ 0 (back-EMF term dominates; confirmed by TF fit)
r = 0.05        # Wheel radius [m]
M = 3.26        # Robot mass [kg]
k = 0.0         # No restoring spring — drivetrain is a double integrator
F_PRE     = 0.0   # No spring preload on a drivetrain
F_COULOMB = 0.4   # Coulomb (rolling + drivetrain) friction force [N]

V_MAX = 10.0   # Supply voltage limit [V]
MOTOR_SIGNAL_MAX = 400
V_TO_SIGNAL = MOTOR_SIGNAL_MAX/V_MAX / 25.4

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
if gamma > 0:
    print(f"  Open-loop: wn = {np.sqrt(gamma):.3f} rad/s, zeta = {beta / (2 * np.sqrt(gamma)):.3f}")
else:
    print(f"  Open-loop: double integrator (no spring), back-EMF damping beta = {beta:.5f}")
print(f"  Free-running no-load speed @ V_MAX: {alpha / beta * V_MAX:.3f} m/s  "
      f"({alpha / beta * V_MAX * 39.37:.1f} in/s)")



# ── 2. OPTIMISATION SETUP ──────────────────────────────────────────────────

# Ramp rates in METERS/SECOND — slow, medium, and fast traversal
RAMP_RATES  = [0.10, 0.25, 0.50]   # m/s
RAMP_MAX_POS = 1.0                  # plateau the ramp saturates at [m]

W_LAG       = 2.0    # steady-state tracking lag (replaces rise-time weight)
W_SETTLE    = 2.0
W_EFFORT    = 6.0
W_IAE       = 4.0    # Integral of Absolute Error (penalises cumulative tracking error)
DT_SIM      = 0.004
T_END       = 5.0     # long enough to reach plateau for the slowest ramp

bounds = [
    (0.0, 10000.0),   # Kp
    (0.0, 10000.0),   # Ki
    (0.0, 10000.0),   # Kd
    (0.0,  300.0),   # a
]

# Starting gains derived from the plant.
# For a double integrator, a reasonable Kp is one that gives a closed-loop
# bandwidth of ~2 rad/s:  Kp ≈ wc² / alpha
wc  = 2.0                         # target crossover [rad/s]
Kp0 = 200 / V_TO_SIGNAL
Ki0 = 100 / V_TO_SIGNAL 
Kd0 = 8 / V_TO_SIGNAL           # critical damping approximation
a0 = 0.0                        #feed forward term

starts = np.array([
    [Kp0,   Ki0,  Kd0,   0.0],
    [Kp0*2, Ki0,  Kd0,   0.0],
    [Kp0,   Ki0,  Kd0*2, 0.0],
    [Kp0/2, Ki0,  Kd0/2, 0.0],
    [Kp0,   1.0,  Kd0,   0.0],
    [Kp0*10, 2.0,  Kd0*3, 0.0],
    [Kp0, 2000.0,  Kd0*3, 0.0],
    [0, 0.0, 0.0, 0.0],
])


# ── 3. RUN OPTIMISATION ────────────────────────────────────────────────────

best_gains, best_j, all_results = run_multistart_optimization(
    starts=starts,
    bounds=bounds,
    ramp_rates=RAMP_RATES,
    ramp_max_pos=RAMP_MAX_POS,
    alpha=alpha, beta=beta, gamma=gamma, v_max=V_MAX,
    w_lag=W_LAG, w_ramp_settle=W_SETTLE, w_effort=W_EFFORT, w_iae=W_IAE,
    dt_sim=DT_SIM, t_end=T_END,
    a_pre=A_PRE, a_coulomb=A_COULOMB,
    deriv_on_error=True,
    max_workers=os.cpu_count() or 1,
)

Kp, Ki, Kd, a = best_gains

print("\n=== Optimised Gains ===")
print(f"  Kp = {Kp:.4f}")
print(f"  Ki = {Ki:.4f}")
print(f"  Kd = {Kd:.4f}")
print(f"  a  = {a:.4f}")
print(f"  Cost J = {best_j:.5f}")

print("\n=== Initial Gains In motor signal units ===")
print(f"  Kp = {Kp0 * V_TO_SIGNAL:.4f}")
print(f"  Ki = {Ki0 * V_TO_SIGNAL:.4f}")
print(f"  Kd = {Kd0 * V_TO_SIGNAL:.4f}")
print(f"  a  = {a0 *  V_TO_SIGNAL:.4f}")
print(f"  Cost J = {best_j:.5f}")

print("\n=== Optimised Gains In motor signal units ===")
print(f"  Kp = {Kp * V_TO_SIGNAL:.4f}")
print(f"  Ki = {Ki * V_TO_SIGNAL:.4f}")
print(f"  Kd = {Kd * V_TO_SIGNAL:.4f}")
print(f"  a  = {a *  V_TO_SIGNAL:.4f}")
print(f"  Cost J = {best_j:.5f}")


# ── 4. COMPARE: INITIAL vs OPTIMISED ──────────────────────────────────────

gains_init = np.array([Kp0, Ki0, Kd0, 0.0])
gains_opt = best_gains

rate_labels = [f"{r:.2f} m/s" for r in RAMP_RATES]

print("\n=== Ramp Tracking Metrics ===")
print(f"{'Config':<12}  {'Rate':<9}  {'IAE(m·s)':<10}  {'Lag(m)':<10}  {'Settle(s)':<11}  {'PeakErr(m)':<12}  {'Vpeak(V)':<10}")
print("-" * 85)

for label, g in [("Initial", gains_init), ("Optimised", gains_opt)]:
    for rate, rl in zip(RAMP_RATES, rate_labels):
        t, x, v, xref_arr = simulate_euler_ramp(
            g, rate, RAMP_MAX_POS, alpha, beta, gamma, V_MAX, DT_SIM, T_END,
            a_pre=A_PRE, a_coulomb=A_COULOMB, deriv_on_error=True,
        )
        lag, settle, peak_err, iae = get_ramp_metrics(t, x, xref_arr)
        print(
            f"{label:<12}  {rl:<9}  "
            f"{iae:10.4f}  {lag:10.4f}  {settle:11.3f}  {peak_err:12.4f}  {np.max(np.abs(v)):10.2f}"
        )


# ── 5. PLOTS ───────────────────────────────────────────────────────────────

# Plot 1: Ramp tracking comparison — initial vs optimised
plt.figure("Ramp Tracking: Initial vs Optimised", figsize=(9, 5.2))
colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
for s, (rate, rl) in enumerate(zip(RAMP_RATES, rate_labels)):
    c = colors[s % len(colors)]
    t, x_i, _, xref_arr = simulate_euler_ramp(
        gains_init, rate, RAMP_MAX_POS, alpha, beta, gamma, V_MAX, DT_SIM, T_END,
        a_pre=A_PRE, a_coulomb=A_COULOMB, deriv_on_error=True,
    )
    _, x_o, _, _ = simulate_euler_ramp(
        gains_opt, rate, RAMP_MAX_POS, alpha, beta, gamma, V_MAX, DT_SIM, T_END,
        a_pre=A_PRE, a_coulomb=A_COULOMB, deriv_on_error=True,
    )
    plt.plot(t, xref_arr, ":", linewidth=1.0, color=c, label=f"Ref {rl}")
    plt.plot(t, x_i, "--", linewidth=1.5, color=c, label=f"Initial {rl}")
    plt.plot(t, x_o, "-", linewidth=2.2, color=c, label=f"Optimised {rl}")

plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Drivetrain PIDF Ramp Tracking — Initial (dashed) vs Optimised (solid)")
plt.legend(loc="lower right", ncol=3, fontsize=8)
plt.grid(True)
plt.xlim(0, T_END)

# Plot 2: Control voltage — optimised, fastest ramp
plt.figure("Control Voltage", figsize=(9, 3.8))
rate_fast = RAMP_RATES[-1]
t, x_oL, v_oL, xref_arr_L = simulate_euler_ramp(
    gains_opt, rate_fast, RAMP_MAX_POS, alpha, beta, gamma, V_MAX, DT_SIM, T_END,
    a_pre=A_PRE, a_coulomb=A_COULOMB, deriv_on_error=True,
)

ax1 = plt.gca()
ax1.plot(t, x_oL, linewidth=2, label="x(t)")
ax1.plot(t, xref_arr_L, ":", linewidth=1.2, label=f"x_ref ({rate_fast:.2f} m/s ramp)")
ax1.set_ylabel("Position (m)")
ax1.set_xlabel("Time (s)")
ax1.set_title(f"Optimised PIDF — {rate_fast:.2f} m/s ramp: Position and Control Voltage")
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

# Plot 3: Cost landscape — Kp vs Kd slice (ramp cost)
minKp = min( Kp0, Kp )
maxKp = max( Kp0, Kp )

minKd = min(Kd0, Kd)
maxKd = max(Kd0, Kd)

kp_span = max(maxKp - minKp, max(abs(Kp0), abs(Kp), 1.0) * 0.1)
kd_span = max(maxKd - minKd, max(abs(Kd0), abs(Kd), 1.0) * 0.1)

Kp_range = np.linspace(max(0.0, minKp - 0.25 * kp_span), maxKp + 0.25 * kp_span, 40)
Kd_range = np.linspace(max(0.0, minKd - 0.25 * kd_span), maxKd + 0.25 * kd_span, 40)
KP_g, KD_g = np.meshgrid(Kp_range, Kd_range)
J_map = np.zeros_like(KP_g)
grid_workers = os.cpu_count() or 1

print(f"Computing cost landscape (40x40 grid, {grid_workers} workers)...")
grid_tasks = [
    (
        kp_val,
        Ki,
        kd_val,
        a,
        RAMP_RATES,
        RAMP_MAX_POS,
        alpha,
        beta,
        gamma,
        V_MAX,
        W_LAG,
        W_SETTLE,
        W_EFFORT,
        DT_SIM,
        T_END,
        A_PRE,
        A_COULOMB,
        True,
        W_IAE,
    )
    for kp_val, kd_val in np.column_stack((KP_g.ravel(), KD_g.ravel()))
]
J_map[:, :] = np.array(evaluate_ramp_cost_grid(grid_tasks, max_workers=grid_workers)).reshape(KP_g.shape)

J_map_safe = np.maximum(J_map, 1e-9)
finite_costs = J_map_safe[np.isfinite(J_map_safe)]
plt.figure("Cost Landscape", figsize=(7.8, 5.4))
try:
    log_vmin = finite_costs.min()
    log_vmax = np.percentile(finite_costs, 98.0)
    if log_vmax <= log_vmin:
        log_vmax = finite_costs.max()
    log_levels = np.geomspace(log_vmin, log_vmax, 60)
    contour = plt.contourf(
        KP_g, KD_g, J_map_safe, levels=log_levels,
        norm=plt.matplotlib.colors.LogNorm(vmin=log_vmin, vmax=log_vmax),
        extend="max",
    )
except Exception:
    contour = plt.contourf(KP_g, KD_g, J_map_safe, levels=60)
plt.colorbar(contour)
plt.plot(Kp, Kd, marker="*", markersize=14, linewidth=0, label="Optimum")
plt.plot(Kp0, Kd0, marker="x", markersize=12, linewidth=0, label="Initial")
plt.xlabel("K_p")
plt.ylabel("K_d")
plt.title(f"Cost landscape (K_i={Ki:.3f}, a={a:.3f} fixed at optimum)")
plt.legend(loc="upper right")

# Plot 4: 3D cost-volume isosurfaces (Kp, Ki, Kd with a fixed)
minKi = min(Ki0, Ki)
maxKi = max(Ki0, Ki)
ki_span = max(maxKi - minKi, max(abs(Ki0), abs(Ki), 1.0) * 0.1)

KI_N = 22
Ki_range = np.linspace(max(0.0, minKi - 0.25 * ki_span), maxKi + 0.25 * ki_span, KI_N)

print(f"Computing 3D cost volume ({len(Kp_range)}x{len(Ki_range)}x{len(Kd_range)} grid, {grid_workers} workers)...")
kp_3d, ki_3d, kd_3d = np.meshgrid(Kp_range, Ki_range, Kd_range, indexing="ij")
volume_tasks = [
    (
        kp_val,
        ki_val,
        kd_val,
        a,
        RAMP_RATES,
        RAMP_MAX_POS,
        alpha,
        beta,
        gamma,
        V_MAX,
        W_LAG,
        W_SETTLE,
        W_EFFORT,
        DT_SIM,
        T_END,
        A_PRE,
        A_COULOMB,
        True,
        W_IAE,
    )
    for kp_val, ki_val, kd_val in np.column_stack((kp_3d.ravel(), ki_3d.ravel(), kd_3d.ravel()))
]
J_vol = np.array(evaluate_ramp_cost_grid(volume_tasks, max_workers=grid_workers)).reshape(kp_3d.shape)
J_vol_safe = np.maximum(J_vol, 1e-9)

try:
    import importlib

    measure = importlib.import_module("skimage.measure")

    finite_vol = J_vol_safe[np.isfinite(J_vol_safe)]
    iso_levels = np.unique(np.percentile(finite_vol, [20.0, 35.0, 50.0]))
    fig_iso = plt.figure("Cost Isosurfaces (Kp, Ki, Kd)", figsize=(8.6, 6.2))
    ax_iso = fig_iso.add_subplot(111, projection="3d")

    cmap_iso = plt.cm.viridis
    level_norm = plt.matplotlib.colors.Normalize(vmin=iso_levels.min(), vmax=iso_levels.max())
    for level in iso_levels:
        verts, faces, _, _ = measure.marching_cubes(J_vol_safe, level=level)

        kp_vals = np.interp(verts[:, 0], np.arange(len(Kp_range)), Kp_range)
        ki_vals = np.interp(verts[:, 1], np.arange(len(Ki_range)), Ki_range)
        kd_vals = np.interp(verts[:, 2], np.arange(len(Kd_range)), Kd_range)

        ax_iso.plot_trisurf(
            kp_vals,
            ki_vals,
            kd_vals,
            triangles=faces,
            color=cmap_iso(level_norm(level)),
            alpha=0.25,
            linewidth=0.08,
            antialiased=True,
        )

    ax_iso.scatter([Kp], [Ki], [Kd], marker="*", s=120, c="crimson", label="Optimum")
    ax_iso.scatter([Kp0], [Ki0], [Kd0], marker="x", s=80, c="black", label="Initial")
    ax_iso.set_xlabel("K_p")
    ax_iso.set_ylabel("K_i")
    ax_iso.set_zlabel("K_d")
    ax_iso.set_title(f"Cost Isosurfaces (a={a:.3f} fixed)")
    ax_iso.legend(loc="upper left")
except Exception as exc:
    print(f"Isosurface plot skipped (requires scikit-image): {exc}")

# # Optional alternative (commented out): 2D Kp-Kd landscape with Ki slider
# from matplotlib.widgets import Slider

# fig_slider, ax_slider = plt.subplots(figsize=(8.2, 5.4))
# plt.subplots_adjust(bottom=0.22)
# slider_ki_vals = np.linspace(Ki_range[0], Ki_range[-1], 30)

# initial_tasks = [
#     (
#         kp_val,
#         slider_ki_vals[0],
#         kd_val,
#         a,
#         RAMP_RATES,
#         RAMP_MAX_POS,
#         alpha,
#         beta,
#         gamma,
#         V_MAX,
#         W_LAG,
#         W_SETTLE,
#         W_EFFORT,
#         DT_SIM,
#         T_END,
#         A_PRE,
#         A_COULOMB,
#         True,
#         W_IAE,
#     )
#     for kp_val, kd_val in np.column_stack((KP_g.ravel(), KD_g.ravel()))
# ]
# J_slider = np.array(evaluate_ramp_cost_grid(initial_tasks, max_workers=grid_workers)).reshape(KP_g.shape)
# J_slider = np.maximum(J_slider, 1e-9)
# contour_slider = ax_slider.contourf(KP_g, KD_g, J_slider, levels=40)
# marker_opt, = ax_slider.plot([Kp], [Kd], "r*", markersize=12)
# marker_init, = ax_slider.plot([Kp0], [Kd0], "kx", markersize=10)
# ax_slider.set_title(f"Cost landscape with Ki slider (Ki={slider_ki_vals[0]:.3f})")
# ax_slider.set_xlabel("K_p")
# ax_slider.set_ylabel("K_d")
# cbar_slider = fig_slider.colorbar(contour_slider, ax=ax_slider)

# slider_ax = fig_slider.add_axes([0.16, 0.08, 0.68, 0.04])
# slider_ki = Slider(slider_ax, "K_i", slider_ki_vals[0], slider_ki_vals[-1], valinit=slider_ki_vals[0])

# def update_ki(val):
#     ki_sel = float(val)
#     tasks = [
#         (
#             kp_val,
#             ki_sel,
#             kd_val,
#             a,
#             RAMP_RATES,
#             RAMP_MAX_POS,
#             alpha,
#             beta,
#             gamma,
#             V_MAX,
#             W_LAG,
#             W_SETTLE,
#             W_EFFORT,
#             DT_SIM,
#             T_END,
#             A_PRE,
#             A_COULOMB,
#             True,
#             W_IAE,
#         )
#         for kp_val, kd_val in np.column_stack((KP_g.ravel(), KD_g.ravel()))
#     ]
#     new_map = np.array(evaluate_ramp_cost_grid(tasks, max_workers=grid_workers)).reshape(KP_g.shape)
#     new_map = np.maximum(new_map, 1e-9)

#     ax_slider.clear()
#     ax_slider.contourf(KP_g, KD_g, new_map, levels=40)
#     ax_slider.plot([Kp], [Kd], "r*", markersize=12)
#     ax_slider.plot([Kp0], [Kd0], "kx", markersize=10)
#     ax_slider.set_xlabel("K_p")
#     ax_slider.set_ylabel("K_d")
#     ax_slider.set_title(f"Cost landscape with Ki slider (Ki={ki_sel:.3f})")
#     fig_slider.canvas.draw_idle()

# slider_ki.on_changed(update_ki)

# Plot 5: Robustness to robot mass (payload variation)
plt.figure("Robustness to Mass", figsize=(7.8, 3.8))
mass_vals = [M * 0.5, M, M * 1.5, M * 2.0]
rate_test = RAMP_RATES[1]   # medium ramp rate

for mm in mass_vals:
    M_eff_k = mm + Jm * Nr**2
    alpha_k  = (Kt * N) / (r * R * M_eff_k)
    beta_k   = (Kt * Kb * Nr**2) / (R * M_eff_k)
    t, x_k, _, xref_k = simulate_euler_ramp(
        gains_opt, rate_test, RAMP_MAX_POS, alpha_k, beta_k, 0.0, V_MAX, DT_SIM, T_END,
        a_pre=F_COULOMB / M_eff_k, a_coulomb=F_COULOMB / M_eff_k, deriv_on_error=True,
    )
    plt.plot(t, x_k, linewidth=2, label=f"M = {mm:.2f} kg")

# Reference ramp (same for all masses)
t_ref = np.arange(int(round(T_END / DT_SIM))) * DT_SIM
xref_ref = np.minimum(rate_test * t_ref, RAMP_MAX_POS)
plt.plot(t_ref, xref_ref, "k:", linewidth=1.2, label="Reference")

plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title(f"Robustness: Optimised Gains Across Robot Masses ({rate_test:.2f} m/s ramp)")
plt.legend(loc="lower right")
plt.grid(True)
plt.xlim(0, T_END)

print("\nDone - 5 figures generated (if scikit-image is installed).")
plt.show()