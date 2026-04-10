%% PIDF Gradient Descent Optimisation — Motor-Rack-Spring System
%  Pololu 37D 70:1 12V, rack-and-pinion (r=35.2mm), spring k=100N/m
%
%  Objective (multi-step, weighted):
%    J = 4·rise_time + 3·(overshoot/5%)² + 2·settle_time/2 + 1·effort
%    evaluated across steps: 5mm, 25mm, 50mm
%
%  Controller: V = Kp·e + Ki·∫e dt − Kd·ẋ + a·x_ref
%              derivative on output (no kick), feed-forward a≥0
%
%  Optimisation: gradient descent via fmincon (L-BFGS-B equivalent),
%                multi-start to escape local minima

clear; clc; close all;

%% ── 1. SYSTEM PARAMETERS ─────────────────────────────────────────────────
R  = 2.31;      % Armature resistance [Ω]
Kt = 0.0109;    % Torque constant [N·m/A]
Kb = 0.0109;    % Back-EMF constant [V·s/rad]
N  = 70;        % Gear ratio
Jm = 5e-6;      % Motor inertia [kg·m²]
b  = 1e-4;      % Motor viscous damping [N·m·s/rad]
r  = 0.0352;    % Pinion pitch radius [m]
M  = 0.085;     % Load + rack mass [kg]
k  = 100;       % Spring constant [N/m]  ← change this to explore
V_MAX = 12.0;   % Supply voltage limit [V]

% Lumped coefficients
Nr    = N / r;
M_eff = M + Jm * Nr^2;
b_eff = b * Nr^2;
alpha = (Kt * N) / (r * R * M_eff);
beta  = b_eff / M_eff + (Kt * Kb * Nr^2) / (R * M_eff);
gamma = k / M_eff;

fprintf('=== Plant ===\n');
fprintf('  M_eff = %.4f kg  (%.1f%% reflected inertia)\n', M_eff, 100*Jm*Nr^2/M_eff);
fprintf('  alpha = %.5f | beta = %.5f | gamma = %.5f\n', alpha, beta, gamma);
fprintf('  Open-loop: wn = %.3f rad/s, zeta = %.3f\n', sqrt(gamma), beta/(2*sqrt(gamma)));

%% ── 2. SIMULATION FUNCTION ────────────────────────────────────────────────
% Fast fixed-step Euler integrator — same approach used in Python optimiser.
% States: x (pos), xdot (vel), xi (integral of error)
function [t, x_out, V_out] = simulate_euler(gains, xref, alpha, beta, gamma, V_MAX, dt, t_end)
    Kp = gains(1); Ki = gains(2); Kd = gains(3); a = gains(4);
    n  = round(t_end / dt);
    x_out = zeros(1, n);
    V_out = zeros(1, n);
    x = 0; xdot = 0; xi = 0;
    for i = 1:n
        V = Kp*(xref-x) + Ki*xi - Kd*xdot + a*xref;
        V = max(-V_MAX, min(V_MAX, V));
        x_out(i) = x;
        V_out(i) = V;
        xi   = xi   + (xref - x)*dt;
        xdot = xdot + (alpha*V - beta*xdot - gamma*x)*dt;
        x    = x    + xdot*dt;
    end
    t = (0:n-1)*dt;
end

%% ── 3. METRICS FUNCTION ───────────────────────────────────────────────────
function [rise, overshoot, settle] = get_metrics(t, x, xref)
    dt  = t(2) - t(1);
    tol = 0.02 * xref;

    % Rise time (10% → 90%)
    i10 = find(x >= 0.10*xref, 1, 'first');
    i90 = find(x >= 0.90*xref, 1, 'first');
    if ~isempty(i10) && ~isempty(i90) && i90 > i10
        rise = (i90 - i10) * dt;
    else
        rise = 3.0;
    end

    % Overshoot (fraction)
    overshoot = max(0, (max(x) - xref) / xref);

    % Settling time
    outside = find(abs(x - xref) > tol);
    if ~isempty(outside)
        settle = t(outside(end));
    else
        settle = 0;
    end
end

%% ── 4. COST FUNCTION ──────────────────────────────────────────────────────
STEPS       = [0.005, 0.025, 0.050];   % 5mm, 25mm, 50mm
W_RISE      = 4.0;
W_OVER      = 3.0;
W_SETTLE    = 2.0;
W_EFFORT    = 1.0;
DT_SIM      = 0.004;    % Euler step [s]
T_END       = 3.0;      % simulation horizon [s]

function J = cost_fn(gains, STEPS, alpha, beta, gamma, V_MAX, ...
                     W_RISE, W_OVER, W_SETTLE, W_EFFORT, DT_SIM, T_END)
    J = 0;
    for s = 1:length(STEPS)
        xref = STEPS(s);
        [t, x, V] = simulate_euler(gains, xref, alpha, beta, gamma, V_MAX, DT_SIM, T_END);
        [rise, over, settle] = get_metrics(t, x, xref);
        effort = mean(V.^2) / (V_MAX^2);
        J = J + W_RISE*(rise/1.0) + W_OVER*(over/0.05)^2 + W_SETTLE*(settle/2.0) + W_EFFORT*effort;
    end
    J = J / length(STEPS);
end

%% ── 5. GRADIENT DESCENT OPTIMISATION ─────────────────────────────────────
% fmincon with 'interior-point' (uses gradient estimation like L-BFGS-B)
% Multi-start to escape local minima

% Wrap cost for fmincon
obj = @(g) cost_fn(g, STEPS, alpha, beta, gamma, V_MAX, ...
                   W_RISE, W_OVER, W_SETTLE, W_EFFORT, DT_SIM, T_END);

lb = [0,    0,   0,  0   ];   % lower bounds: all gains ≥ 0
ub = [5000, 30000, 300, 3000]; % upper bounds

% Starting points (from pole placement + known-good Python seeds)
p0 = 3 * sqrt(gamma);
Kp0 = (3*p0^2 - gamma) / alpha;
Ki0 = p0^3 / alpha;
Kd0 = max(0, (3*p0 - beta) / alpha);

starts = [
    Kp0,    Ki0,   Kd0,   0;      % Pole-placement baseline
    1549,   9.6,   31.3,  0;      % Pre-validated from Python
    1596.9, 1.8,   32.5,  7.6;    % Python optimised (best known)
    1200,   30,    25,    0;
    900,    5,     45,    0;
    2000,   100,   50,    100;
];

opts = optimoptions('fmincon', ...
    'Algorithm',          'interior-point', ...
    'Display',            'off', ...
    'MaxIterations',      200, ...
    'MaxFunctionEvaluations', 5000, ...
    'OptimalityTolerance',    1e-8, ...
    'StepTolerance',          1e-9, ...
    'FiniteDifferenceStepSize', 2.0, ...  % larger FD step → faster gradient est.
    'FiniteDifferenceType',     'central');

best_J     = inf;
best_gains = starts(3,:);   % pre-seed with known-good result
all_results = zeros(size(starts,1), 5);  % [Kp Ki Kd a J]

fprintf('\n=== Gradient Descent (multi-start) ===\n');
for i = 1:size(starts,1)
    try
        [g_opt, J_opt] = fmincon(obj, starts(i,:), [], [], [], [], lb, ub, [], opts);
        all_results(i,:) = [g_opt, J_opt];
        status = '';
        if J_opt < best_J
            best_J     = J_opt;
            best_gains = g_opt;
            status = '  ← best';
        end
        fprintf('  Start %d: J=%.5f  Kp=%.1f Ki=%.3f Kd=%.3f a=%.3f%s\n', ...
                i, J_opt, g_opt(1), g_opt(2), g_opt(3), g_opt(4), status);
    catch
        fprintf('  Start %d: failed\n', i);
    end
end

Kp = best_gains(1);
Ki = best_gains(2);
Kd = best_gains(3);
a  = best_gains(4);

fprintf('\n=== Optimised Gains ===\n');
fprintf('  Kp = %.4f\n', Kp);
fprintf('  Ki = %.4f\n', Ki);
fprintf('  Kd = %.4f\n', Kd);
fprintf('  a  = %.4f\n', a);
fprintf('  Cost J = %.5f\n', best_J);

%% ── 6. COMPARE: INITIAL vs OPTIMISED ─────────────────────────────────────
gains_init = [Kp0, Ki0, Kd0, 0];
gains_opt  = best_gains;

fprintf('\n=== Step Response Metrics ===\n');
fprintf('%-12s  %-6s  %-8s  %-8s  %-10s  %-10s\n', 'Config','Step','Rise(s)','OS(%)', 'Settle(s)','Vpeak(V)');
fprintf('%s\n', repmat('-',1,65));

for cfg = 1:2
    if cfg==1; g=gains_init; label='Initial';
    else;      g=gains_opt;  label='Optimised'; end
    for s=1:length(STEPS)
        xref=STEPS(s);
        [t,x,V]=simulate_euler(g,xref,alpha,beta,gamma,V_MAX,DT_SIM,T_END);
        [rise,over,settle]=get_metrics(t,x,xref);
        fprintf('%-12s  %4.0fmm  %8.3f  %7.2f%%  %10.3f  %10.2f\n', ...
                label,xref*1000,rise,over*100,settle,max(abs(V)));
    end
end

%% ── 7. PLOTS ──────────────────────────────────────────────────────────────
T_PLOT = linspace(0, T_END, round(T_END/DT_SIM));
colors_init = {'b--','r--','g--'};
colors_opt  = {'b-', 'r-', 'g-'};
step_labels = {'5mm','25mm','50mm'};

% ── Plot 1: Step response comparison ──
figure('Name','Step Response: Initial vs Optimised','Position',[50 50 900 520]);
hold on;
h_ref = yline(0,'k:','LineWidth',1.2);   % placeholder; overwritten in loop
for s = 1:length(STEPS)
    xref = STEPS(s);
    [t,x_i,~] = simulate_euler(gains_init,xref,alpha,beta,gamma,V_MAX,DT_SIM,T_END);
    [~,x_o,~] = simulate_euler(gains_opt, xref,alpha,beta,gamma,V_MAX,DT_SIM,T_END);
    plot(t, x_i*1000, colors_init{s}, 'LineWidth',1.5, ...
         'DisplayName', sprintf('Initial — %s', step_labels{s}));
    plot(t, x_o*1000, colors_opt{s},  'LineWidth',2.2, ...
         'DisplayName', sprintf('Optimised — %s', step_labels{s}));
    yline(xref*1000, ':', 'Color',[0.5 0.5 0.5], 'LineWidth',0.8, 'HandleVisibility','off');
end
xlabel('Time (s)'); ylabel('Position (mm)');
title('PIDF Step Response — Initial (dashed) vs Gradient-Descent Optimised (solid)');
legend('Location','southeast','NumColumns',2); grid on;
xlim([0 T_END]); ylim([-3 60]);

% ── Plot 2: Control voltage (optimised, 50mm step) ──
figure('Name','Control Voltage','Position',[50 600 900 380]);
[t,x_o50,V_o50] = simulate_euler(gains_opt,0.050,alpha,beta,gamma,V_MAX,DT_SIM,T_END);
yyaxis left;
plot(t, x_o50*1000, 'b-', 'LineWidth',2); hold on;
yline(50,'b:','LineWidth',1);
ylabel('Position (mm)'); ylim([-2 58]);
yyaxis right;
plot(t, V_o50, 'r-', 'LineWidth',1.5);
yline( V_MAX,'r:','LineWidth',1);
yline(-V_MAX,'r:','LineWidth',1);
ylabel('Voltage V(t) [V]'); ylim([-14 14]);
xlabel('Time (s)');
title('Optimised PIDF — 50mm Step: Position and Control Voltage');
legend('x(t)','x_{ref}','V(t)','±V_{max}','Location','east'); grid on;

% ── Plot 3: Cost landscape — Kp vs Kd slice (Ki, a fixed at optimum) ──
figure('Name','Cost Landscape','Position',[980 50 780 540]);
Kp_range = linspace(500, 3000, 40);
Kd_range = linspace(0,   80,   40);
[KP_g, KD_g] = meshgrid(Kp_range, Kd_range);
J_map = zeros(size(KP_g));
fprintf('Computing cost landscape (40×40 grid)...\n');
for ii=1:numel(KP_g)
    g=[KP_g(ii), Ki, KD_g(ii), a];
    J_map(ii) = cost_fn(g,STEPS,alpha,beta,gamma,V_MAX,W_RISE,W_OVER,W_SETTLE,W_EFFORT,DT_SIM,T_END);
end
contourf(KP_g, KD_g, J_map, 25, 'LineColor','none'); colorbar;
hold on;
plot(Kp, Kd, 'w*', 'MarkerSize',14, 'LineWidth',2.5, 'DisplayName','Optimum');
plot(Kp0, Kd0, 'wx', 'MarkerSize',12, 'LineWidth',2.0, 'DisplayName','Initial');
xlabel('K_p'); ylabel('K_d');
title(sprintf('Cost landscape (K_i=%.2f, a=%.2f fixed at optimum)', Ki, a));
legend('Location','northeast'); colormap('hot');
set(gca,'ColorScale','log');   % log scale reveals fine structure near optimum

% ── Plot 4: Spring constant robustness (with optimised gains, no re-tuning) ──
figure('Name','Robustness to k','Position',[980 620 780 380]);
k_vals   = [50, 100, 200, 400];
k_colors = {'b','r','g','m'};
hold on;
xref_test = 0.025;   % 25mm step
for idx = 1:length(k_vals)
    kk = k_vals(idx);
    gamma_k = kk / M_eff;
    alpha_k = alpha;      % alpha independent of k
    beta_k  = beta;
    [t,x_k,~] = simulate_euler(gains_opt,xref_test,alpha_k,beta_k,gamma_k,V_MAX,DT_SIM,T_END);
    plot(t, x_k*1000, k_colors{idx}, 'LineWidth',2, ...
         'DisplayName', sprintf('k = %d N/m', kk));
end
yline(xref_test*1000,'k:','LineWidth',1.2,'DisplayName','Reference');
xlabel('Time (s)'); ylabel('Position (mm)');
title('Robustness: Optimised Gains Applied Across Spring Constants (25mm step)');
legend('Location','southeast'); grid on; xlim([0 T_END]);

fprintf('\nDone — 4 figures generated.\n');