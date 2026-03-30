"""
pidf_optimizer.py — Abstract PIDF optimisation engine

Supports two ways to specify the plant:

  1. Lumped second-order parameters (original interface):
       x'' = alpha·V − beta·x' − gamma·x − a_pre − a_coulomb·sign(x')
       Pass: alpha, beta, gamma to cost_fn / run_multistart_optimization.

  2. Transfer function coefficient arrays (MATLAB-style):
       Y(s)/V(s) = N(s)/D(s)
       where num=[b0, b1, ...] and den=[a0, a1, ...] are coefficients in
       descending powers of s (same convention as MATLAB tf() and
       scipy.signal.tf2ss).
       Pass: num=..., den=... to cost_fn / run_multistart_optimization.
       The plant must be proper (deg(num) < deg(den)).

Disturbances (a_pre, a_coulomb) are physical acceleration terms:
    a_pre     : constant disturbance = F_preload / M_eff  [m/s²]
    a_coulomb : Coulomb friction magnitude = F_c / M_eff  [m/s²]
For the TF path these are scaled via the input-to-acceleration gain (C·A·B)
and only apply meaningfully to second-order mechanical plants.

The PIDF controller (derivative on output, no derivative kick):
    V = Kp·e + Ki·∫e dt − Kd·ẏ + a·x_ref
    e = x_ref − y

Usage: import and call run_multistart_optimization().
"""

import os
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

import numpy as np
from scipy.optimize import minimize


# ── SIMULATION ─────────────────────────────────────────────────────────────

def simulate_euler(gains, xref, alpha, beta, gamma, v_max, dt, t_end,
                   a_pre=0.0, a_coulomb=0.0, deriv_on_error=False):
    """
    Fixed-step Euler integrator for the PIDF-controlled second-order plant.

    Parameters
    ----------
    gains          : [Kp, Ki, Kd, a]
    xref           : scalar reference position
    alpha          : plant input gain  (e.g. Kt·N / (r·R·M_eff))
    beta           : plant damping     (e.g. b_eff/M_eff + Kt·Kb·Nr²/(R·M_eff))
    gamma          : plant stiffness   (e.g. k / M_eff)
    v_max          : actuator saturation limit
    dt             : timestep [s]
    t_end          : simulation duration [s]
    a_pre          : constant disturbance acceleration = F_preload / M_eff [m/s²]
                     Positive value opposes positive displacement.
    a_coulomb      : Coulomb friction deceleration magnitude = F_c / M_eff [m/s²]
                     Applied as -a_coulomb·sign(x'), always opposes motion.
    deriv_on_error : if True, Kd acts on d(error)/dt  (standard PID).
                     if False (default), Kd acts on -dx/dt  (derivative on
                     measurement — eliminates derivative kick on setpoint changes).

    Returns
    -------
    t      : time vector  (n,)
    x_out  : position history (n,)
    v_out  : control signal history (n,)
    """
    kp, ki, kd, a = gains
    n = int(round(t_end / dt))

    x_out = np.zeros(n)
    v_out = np.zeros(n)

    x    = 0.0
    xdot = 0.0
    xi   = 0.0
    e_prev = xref   # used only when deriv_on_error=True

    for i in range(n):
        e = xref - x
        if deriv_on_error:
            edot = (e - e_prev) / dt if i > 0 else 0.0
            v = kp * e + ki * xi + kd * edot + a * xref
            e_prev = e
        else:
            v = kp * e + ki * xi - kd * xdot + a * xref
        v = np.clip(v, -v_max, v_max)

        x_out[i] = x
        v_out[i] = v

        xi   += e * dt
        xdot += (
            alpha * v
            - beta * xdot
            - gamma * x
            - a_pre
            - a_coulomb * np.sign(xdot)
        ) * dt
        x += xdot * dt

    t = np.arange(n) * dt
    return t, x_out, v_out


def simulate_euler_ramp(gains, ramp_rate, max_pos, alpha, beta, gamma, v_max, dt, t_end,
                        a_pre=0.0, a_coulomb=0.0, deriv_on_error=True):
    """
    Fixed-step Euler integrator with a ramp reference: xref(t) = min(ramp_rate·t, max_pos).

    Parameters
    ----------
    gains       : [Kp, Ki, Kd, a]
    ramp_rate   : reference velocity [units/s]  (e.g. m/s)
    max_pos     : reference plateau value — ramp stops here [units]
    alpha, beta, gamma, v_max, dt, t_end, a_pre, a_coulomb : same as simulate_euler
    deriv_on_error : default True for ramp tracking (derivative on error is
                     standard and avoids lag from filtering measurement noise).

    Returns
    -------
    t        : time vector  (n,)
    x_out    : position history (n,)
    v_out    : control signal history (n,)
    xref_out : reference trajectory history (n,)
    """
    kp, ki, kd, a = gains
    n = int(round(t_end / dt))

    x_out    = np.zeros(n)
    v_out    = np.zeros(n)
    xref_out = np.zeros(n)

    x      = 0.0
    xdot   = 0.0
    xi     = 0.0
    e_prev = 0.0

    for i in range(n):
        t_now = i * dt
        xref  = min(ramp_rate * t_now, max_pos)
        e     = xref - x

        if deriv_on_error:
            edot = (e - e_prev) / dt if i > 0 else 0.0
            v = kp * e + ki * xi + kd * edot + a * xref
            e_prev = e
        else:
            v = kp * e + ki * xi - kd * xdot + a * xref
        v = np.clip(v, -v_max, v_max)

        x_out[i]    = x
        v_out[i]    = v
        xref_out[i] = xref

        xi   += e * dt
        xdot += (
            alpha * v
            - beta * xdot
            - gamma * x
            - a_pre
            - a_coulomb * np.sign(xdot)
        ) * dt
        x += xdot * dt

    t = np.arange(n) * dt
    return t, x_out, v_out, xref_out

# ── RAMP METRICS ───────────────────────────────────────────────────────────

def get_ramp_metrics(t, x, xref_arr):
    """
    Performance metrics for ramp-reference tracking.

    Unlike step metrics, overshoot and rise time are not meaningful here.
    Instead we report:

    Parameters
    ----------
    t        : time vector (n,)
    x        : position history (n,)
    xref_arr : reference trajectory history (n,)  — from simulate_euler_ramp

    Returns
    -------
    lag       : mean absolute tracking error over the final 30% of the sim [units]
                (steady-state ramp lag after the transient has died out)
    settle    : time at which |error| first drops below 2% of max_pos and stays
                there [s].  Returns t_end if it never settles.
    peak_err  : maximum absolute tracking error over the whole run [units]
    iae       : Integral of Absolute Error = ∫|e(t)| dt ≈ Σ|e|·Δt [units·s]
    """
    err      = xref_arr - x
    max_pos  = xref_arr[-1]            # plateau value
    tol      = 0.02 * max_pos if max_pos > 0 else 0.01
    dt       = t[1] - t[0]

    # Steady-state lag: average |error| over last 30% of simulation
    tail_start = int(0.70 * len(t))
    lag = np.mean(np.abs(err[tail_start:]))

    # Settling time: last sample where |error| > tol
    outside = np.where(np.abs(err) > tol)[0]
    settle  = t[outside[-1]] if len(outside) > 0 else 0.0

    peak_err = np.max(np.abs(err))

    # IAE: integral of |error| over full run
    iae = np.sum(np.abs(err)) * dt

    return lag, settle, peak_err, iae


def cost_fn_ramp(gains, ramp_rates, max_pos, alpha, beta, gamma, v_max,
                 w_lag, w_settle, w_effort, dt_sim, t_end,
                 a_pre=0.0, a_coulomb=0.0, deriv_on_error=True, w_iae=0.0):
    """
    Weighted multi-rate ramp-tracking cost:
        J = w_lag·(lag/max_pos) + w_settle·(settle/t_end)
          + w_effort·RMS² + w_iae·(IAE/(max_pos·t_end))
    averaged over all ramp rates.

    lag_ref is the theoretical steady-state lag of a pure integrator at
    the given rate, used as a normalising reference:
        lag_ref = ramp_rate / (Kp · alpha)   ← first-order approximation
    Since gains are being optimised, we instead normalise by
    lag_ref = ramp_rate · dt_sim  (one timestep of lag — a practical floor).
    """
    j = 0.0
    for rate in ramp_rates:
        t, x, v, xref_arr = simulate_euler_ramp(
            gains, rate, max_pos, alpha, beta, gamma, v_max, dt_sim, t_end,
            a_pre=a_pre, a_coulomb=a_coulomb, deriv_on_error=deriv_on_error,
        )
        lag, settle, _, iae = get_ramp_metrics(t, x, xref_arr)
        effort = np.mean(v**2) / (v_max**2)

        # Normalise each term to be dimensionless
        j += (
            w_lag    * (lag    / max_pos)
            + w_settle * (settle / t_end)
            + w_effort * effort
            + w_iae    * (iae   / (max_pos * t_end))
        )
    return j / len(ramp_rates)


# ── TRANSFER FUNCTION UTILITIES ──────────────────────────────────────────────────────

def tf_to_ss(num, den):
    """
    Convert MATLAB-style transfer function coefficient arrays to state-space.

    Parameters
    ----------
    num : array-like — numerator coefficients, descending powers of s
    den : array-like — denominator coefficients, descending powers of s
          e.g. for K/(tau·s + 1): num=[K], den=[tau, 1]
               for alpha/(s² + beta·s + gamma): num=[alpha], den=[1, beta, gamma]

    Returns
    -------
    A, B, C, D_mat : np.ndarray state-space matrices (controllable canonical form)
    """
    from scipy.signal import tf2ss
    A, B, C, D_mat = tf2ss(np.atleast_1d(num), np.atleast_1d(den))
    return A, B, C, D_mat


def simulate_ss(gains, xref, A, B, C, D_mat, v_max, dt, t_end,
               a_pre=0.0, a_coulomb=0.0):
    """
    Fixed-step Euler integrator for a PIDF-controlled state-space plant.

    The plant is described by:
        state' = A·state + B·V
        y      = C·state          (D must be zero; improper TFs not supported)

    The PIDF derivative term uses a numerical first difference of y, matching
    how embedded controllers typically compute velocity from position.

    Physical disturbance accelerations (a_pre, a_coulomb) are applied to the
    last state derivative scaled by the input-to-acceleration gain C·A·B.
    This is exact for second-order mechanical plants; set both to 0 for other
    system types.

    Parameters
    ----------
    gains   : [Kp, Ki, Kd, a]
    xref    : scalar reference output value
    A, B, C, D_mat : state-space matrices from tf_to_ss
    v_max   : actuator saturation limit
    dt      : timestep [s]
    t_end   : simulation duration [s]
    a_pre   : constant acceleration disturbance [m/s²]
    a_coulomb : Coulomb friction magnitude [m/s²]

    Returns
    -------
    t      : time vector  (n,)
    x_out  : output (position) history (n,)
    v_out  : control signal history (n,)
    """
    D_val = float(D_mat.flat[0])
    if abs(D_val) > 1e-6:
        raise ValueError(
            f"Non-zero D matrix (D={D_val:.4g}): only proper transfer functions "
            "(deg(num) < deg(den)) are supported."
        )

    kp, ki, kd, a_ff = gains
    n_states = A.shape[0]
    n = int(round(t_end / dt))

    # Pre-compute input-to-acceleration gain for disturbance scaling.
    # C·A·B = gain from control input to the second derivative of output.
    # Exact for 2nd-order mechanical plants; disturbances disabled for n<2.
    if n_states >= 2 and (a_pre != 0.0 or a_coulomb != 0.0):
        caab = float((C @ A @ B).flat[0])
        _apply_dist = caab != 0.0
    else:
        caab = 1.0
        _apply_dist = False

    x_out = np.zeros(n)
    v_out = np.zeros(n)

    state = np.zeros(n_states)
    xi = 0.0
    y_prev = 0.0

    for i in range(n):
        y = float((C @ state).flat[0])
        y_dot = (y - y_prev) / dt if i > 0 else 0.0
        y_prev = y

        e = xref - y
        ctrl = kp * e + ki * xi - kd * y_dot + a_ff * xref
        ctrl = np.clip(ctrl, -v_max, v_max)

        x_out[i] = y
        v_out[i] = ctrl

        xi += e * dt

        state_dot = (A @ state).flatten() + (B.flatten() * ctrl)
        if _apply_dist:
            state_dot[-1] -= (a_pre + a_coulomb * np.sign(y_dot)) / caab

        state += state_dot * dt

    t = np.arange(n) * dt
    return t, x_out, v_out

# ── METRICS ────────────────────────────────────────────────────────────────

def get_metrics(t, x, xref):
    """
    Compute step-response performance metrics.

    Returns
    -------
    rise      : 10%→90% rise time [s]  (capped at 3 s if unreachable)
    overshoot : fractional overshoot   (e.g. 0.05 = 5%)
    settle    : time to enter and stay within ±2% band [s]
    """
    dt = t[1] - t[0]
    tol = 0.02 * xref

    i10_arr = np.where(x >= 0.10 * xref)[0]
    i90_arr = np.where(x >= 0.90 * xref)[0]

    if len(i10_arr) > 0 and len(i90_arr) > 0:
        i10 = i10_arr[0]
        i90 = i90_arr[0]
        rise = (i90 - i10) * dt if i90 > i10 else 3.0
    else:
        rise = 3.0

    overshoot = max(0.0, (np.max(x) - xref) / xref)

    outside = np.where(np.abs(x - xref) > tol)[0]
    settle = t[outside[-1]] if len(outside) > 0 else 0.0

    return rise, overshoot, settle


# ── COST FUNCTION ──────────────────────────────────────────────────────────

def cost_fn(gains, steps, alpha, beta, gamma, v_max,
            w_rise, w_over, w_settle, w_effort, dt_sim, t_end,
            a_pre=0.0, a_coulomb=0.0, num=None, den=None):
    """
    Weighted multi-step cost:
        J = w_rise·(rise/1s) + w_over·(over/5%)² + w_settle·(settle/2s) + w_effort·RMS²
    averaged over all reference steps.

    If num and den are provided the plant is simulated via simulate_ss and
    alpha/beta/gamma are ignored.  Otherwise simulate_euler is used.
    """
    if num is not None and den is not None:
        A_ss, B_ss, C_ss, D_ss = tf_to_ss(num, den)
        def _sim(g, xr):
            return simulate_ss(g, xr, A_ss, B_ss, C_ss, D_ss, v_max, dt_sim, t_end,
                               a_pre=a_pre, a_coulomb=a_coulomb)
    else:
        def _sim(g, xr):
            return simulate_euler(g, xr, alpha, beta, gamma, v_max, dt_sim, t_end,
                                  a_pre=a_pre, a_coulomb=a_coulomb)

    j = 0.0
    for xref in steps:
        t, x, v = _sim(gains, xref)
        rise, over, settle = get_metrics(t, x, xref)
        effort = np.mean(v**2) / (v_max**2)

        j += (
            w_rise   * (rise / 1.0)
            + w_over   * (over / 0.05)**2
            + w_settle * (settle / 2.0)
            + w_effort * effort
        )

    return j / len(steps)


def _process_pool_kwargs(max_workers):
    kwargs = {"max_workers": max_workers}
    try:
        kwargs["mp_context"] = get_context("fork")
    except ValueError:
        pass
    return kwargs


def _run_single_optimization(task):
    (
        index,
        x0,
        bounds,
        default_opts,
        use_ramp,
        use_tf,
        steps,
        alpha,
        beta,
        gamma,
        v_max,
        w_rise,
        w_over,
        w_settle,
        w_effort,
        dt_sim,
        t_end,
        a_pre,
        a_coulomb,
        num,
        den,
        deriv_on_error,
        ramp_rates,
        ramp_max_pos,
        w_lag,
        w_ramp_settle,
        w_iae,
    ) = task

    if use_ramp:
        def obj(g):
            return cost_fn_ramp(
                g, ramp_rates, ramp_max_pos,
                alpha, beta, gamma, v_max,
                w_lag, w_ramp_settle, w_effort,
                dt_sim, t_end,
                a_pre=a_pre, a_coulomb=a_coulomb,
                deriv_on_error=deriv_on_error,
                w_iae=w_iae,
            )
    elif use_tf:
        A_ss, B_ss, C_ss, D_ss = tf_to_ss(num, den)

        def obj(g):
            j = 0.0
            for xref in steps:
                t, x, v = simulate_ss(
                    g, xref, A_ss, B_ss, C_ss, D_ss, v_max, dt_sim, t_end,
                    a_pre=a_pre, a_coulomb=a_coulomb,
                )
                rise, over, settle = get_metrics(t, x, xref)
                effort = np.mean(v**2) / (v_max**2)
                j += (
                    w_rise * (rise / 1.0)
                    + w_over * (over / 0.05)**2
                    + w_settle * (settle / 2.0)
                    + w_effort * effort
                )
            return j / len(steps)
    else:
        def obj(g):
            return cost_fn(
                g, steps, alpha, beta, gamma, v_max,
                w_rise, w_over, w_settle, w_effort, dt_sim, t_end,
                a_pre=a_pre, a_coulomb=a_coulomb,
            )

    try:
        result = minimize(obj, x0, method="L-BFGS-B", bounds=bounds, options=default_opts)
        return index, result.x, result.fun, None
    except Exception as exc:
        return index, None, None, str(exc)


def _evaluate_ramp_cost_task(task):
    (
        kp_val,
        ki_val,
        kd_val,
        a_val,
        ramp_rates,
        max_pos,
        alpha,
        beta,
        gamma,
        v_max,
        w_lag,
        w_settle,
        w_effort,
        dt_sim,
        t_end,
        a_pre,
        a_coulomb,
        deriv_on_error,
        w_iae,
    ) = task

    return cost_fn_ramp(
        [kp_val, ki_val, kd_val, a_val],
        ramp_rates,
        max_pos,
        alpha,
        beta,
        gamma,
        v_max,
        w_lag,
        w_settle,
        w_effort,
        dt_sim,
        t_end,
        a_pre=a_pre,
        a_coulomb=a_coulomb,
        deriv_on_error=deriv_on_error,
        w_iae=w_iae,
    )


def evaluate_ramp_cost_grid(tasks, max_workers=None):
    worker_count = max_workers or (os.cpu_count() or 1)
    if worker_count <= 1 or len(tasks) <= 1:
        return [_evaluate_ramp_cost_task(task) for task in tasks]

    with ProcessPoolExecutor(**_process_pool_kwargs(worker_count)) as executor:
        return list(executor.map(_evaluate_ramp_cost_task, tasks))


# ── OPTIMISATION ───────────────────────────────────────────────────────────

def run_multistart_optimization(
    starts,
    bounds,
    steps=None,
    alpha=None, beta=None, gamma=None, v_max=12.0,
    w_rise=4.0, w_over=3.0, w_settle=2.0, w_effort=1.0,
    dt_sim=0.004, t_end=3.0,
    a_pre=0.0, a_coulomb=0.0,
    num=None, den=None,
    deriv_on_error=False,
    ramp_rates=None, ramp_max_pos=None,
    w_lag=4.0, w_ramp_settle=2.0, w_iae=0.0,
    lbfgs_options=None,
    max_workers=None,
):
    """
    Multi-start L-BFGS-B minimisation of cost_fn or cost_fn_ramp.

    Reference modes (mutually exclusive):
      - Step inputs  : provide steps=[...]          → uses cost_fn
      - Ramp inputs  : provide ramp_rates=[...] and ramp_max_pos=<float>
                       → uses cost_fn_ramp

    Plant specification (mutually exclusive):
      - Lumped parameters: alpha, beta, gamma       → simulate_euler / simulate_euler_ramp
      - Transfer function: num, den                 → simulate_ss

    Parameters
    ----------
    starts         : (N, 4) array of initial [Kp, Ki, Kd, a] guesses
    bounds         : list of (lo, hi) tuples, one per gain
    steps          : list of scalar reference positions (step mode)
    alpha, beta, gamma : lumped plant parameters
    v_max          : actuator saturation limit
    w_rise, w_over, w_settle, w_effort : step-mode cost weights
    dt_sim         : simulation timestep [s]
    t_end          : simulation duration [s]
    a_pre          : spring preload disturbance = F_preload / M_eff [m/s²]
    a_coulomb      : Coulomb friction magnitude = F_c / M_eff [m/s²]
    num, den       : TF numerator / denominator coefficient arrays
    deriv_on_error : if True, Kd acts on d(error)/dt; if False (default),
                     Kd acts on -dx/dt (no derivative kick)
    ramp_rates     : list of ramp velocities to evaluate (ramp mode)
    ramp_max_pos   : plateau position the ramp saturates at (ramp mode)
    w_lag          : ramp-mode weight on steady-state tracking lag
    w_ramp_settle  : ramp-mode weight on settling time
    w_iae          : ramp-mode weight on IAE (Integral of Absolute Error)
    lbfgs_options  : dict forwarded to scipy.optimize.minimize
    max_workers    : number of worker threads used for multi-start optimisation.
                     Defaults to all available CPU workers.

    Returns
    -------
    best_gains  : (4,) array — optimal [Kp, Ki, Kd, a]
    best_j      : float      — cost at optimum
    all_results : (N, 5) array — [Kp, Ki, Kd, a, J] per start
    """
    # ── Validate inputs ────────────────────────────────────────────────────
    use_ramp = ramp_rates is not None
    use_tf   = num is not None and den is not None
    use_lump = alpha is not None and beta is not None and gamma is not None

    if use_ramp and ramp_max_pos is None:
        raise ValueError("ramp_max_pos must be provided when ramp_rates is given.")
    if not use_ramp and steps is None:
        raise ValueError("Provide either steps (step mode) or ramp_rates (ramp mode).")
    if not use_tf and not use_lump:
        raise ValueError("Provide either (alpha, beta, gamma) or (num, den) to specify the plant.")
    if use_tf and not use_lump and use_ramp:
        raise ValueError("Ramp mode currently only supports the lumped (alpha, beta, gamma) plant.")

    default_opts = {
        "maxiter": 200,
        "maxfun":  5000,
        "ftol":    1e-8,
        "eps":     2.0,
        "disp":    False,
    }
    if lbfgs_options:
        default_opts.update(lbfgs_options)

    starts = np.asarray(starts)
    all_results = np.zeros((len(starts), 5))
    best_j = np.inf
    best_gains = starts[0].copy()
    worker_count = max_workers or (os.cpu_count() or 1)

    print(f"=== Gradient Descent (multi-start, {worker_count} workers) ===")

    tasks = [
        (
            i,
            x0,
            bounds,
            default_opts,
            use_ramp,
            use_tf,
            steps,
            alpha,
            beta,
            gamma,
            v_max,
            w_rise,
            w_over,
            w_settle,
            w_effort,
            dt_sim,
            t_end,
            a_pre,
            a_coulomb,
            num,
            den,
            deriv_on_error,
            ramp_rates,
            ramp_max_pos,
            w_lag,
            w_ramp_settle,
            w_iae,
        )
        for i, x0 in enumerate(starts, start=1)
    ]

    if worker_count <= 1 or len(tasks) <= 1:
        results = [_run_single_optimization(task) for task in tasks]
    else:
        with ProcessPoolExecutor(**_process_pool_kwargs(worker_count)) as executor:
            results = list(executor.map(_run_single_optimization, tasks))

    results.sort(key=lambda item: item[0])
    for i, g_opt, j_opt, exc in results:
        if exc is not None:
            print(f"  Start {i}: failed")
            continue

        all_results[i - 1] = [g_opt[0], g_opt[1], g_opt[2], g_opt[3], j_opt]

        tag = ""
        if j_opt < best_j:
            best_j = j_opt
            best_gains = g_opt.copy()
            tag = "  <- best"

        print(
            f"  Start {i}: J={j_opt:.5f}  "
            f"Kp={g_opt[0]:.1f} Ki={g_opt[1]:.3f} "
            f"Kd={g_opt[2]:.3f} a={g_opt[3]:.3f}{tag}"
        )

    return best_gains, best_j, all_results
