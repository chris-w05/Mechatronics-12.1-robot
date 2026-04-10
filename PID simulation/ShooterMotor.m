%% Motor-Rack-Spring PIDF Simulation
%  Pololu 37D 70:1 12V Gearmotor driving a rack-and-pinion against a spring
%  Controller: PIDF with derivative on output (avoids derivative kick)
%  Feed-forward: a * x_ref added directly to voltage
%
%  State variables: x1 = x (position), x2 = xdot (velocity), x3 = integral(e)
%  Control law:     V = Kp*e + Ki*integral(e) - Kd*xdot + a*x_ref
%  Closed-loop TF:  X(s)/Xref(s) = alpha*[(Kp+a)*s + Ki] /
%                                   [s^3 + (beta+alpha*Kd)*s^2 +
%                                    (alpha*Kp+gamma)*s + alpha*Ki]

clear; clc; close all;

%% ============================================================
%  SYSTEM PARAMETERS
%% ============================================================
% --- Motor (Pololu 37D 70:1 12V, item #4744) ---
R  = 2.31;       % Armature resistance [Ohm]  (V / I_stall = 12/5.2)
Kt = 0.0109;     % Torque constant [N·m/A]    (motor shaft)
Kb = 0.0109;     % Back-EMF constant [V·s/rad](= Kt in SI)
N  = 70;         % Gear ratio [-]
Jm = 5e-6;       % Motor rotor inertia [kg·m²] (estimated, typical 37D)
b  = 1e-4;       % Motor viscous damping [N·m·s/rad] (estimated)

% --- Mechanical ---
r  = 0.0352/2;     % Pinion pitch radius [m]
M  = 0.085;      % Rack + load mass [kg]
k  = 100;        % Spring constant [N/m]  <-- vary this to explore

%% ============================================================
%  DERIVED QUANTITIES
%% ============================================================
Nr  = N / r;                          % N/r ratio [rad/m]

M_eff = M + Jm * Nr^2;               % Effective mass (reflected inertia) [kg]
b_eff = b * Nr^2;                     % Effective damping [N·s/m]

% Lumped plant coefficients (2nd-order plant: xddot = alpha*V - beta*xdot - gamma*x)
alpha = (Kt * N)  / (r * R * M_eff);
beta  = b_eff/M_eff + (Kt*Kb*Nr^2) / (R * M_eff);
gamma = k / M_eff;

fprintf('=== System Parameters ===\n');
fprintf('  M_eff  = %.4f kg  (dominanted by reflected inertia: Jm*(N/r)^2 = %.3f kg)\n', M_eff, Jm*Nr^2);
fprintf('  alpha  = %.6f\n', alpha);
fprintf('  beta   = %.6f\n', beta);
fprintf('  gamma  = %.6f\n', gamma);
fprintf('  Open-loop wn   = %.4f rad/s (%.3f Hz)\n', sqrt(gamma), sqrt(gamma)/(2*pi));
fprintf('  Open-loop zeta = %.4f (>1 = overdamped due to strong back-EMF)\n', beta/(2*sqrt(gamma)));

%% ============================================================
%  CONTROLLER GAINS
%  Pole placement: place all 3 CL poles at s = -p (triple real pole)
%  Matching coefficients of (s+p)^3 = s^3 + 3p*s^2 + 3p^2*s + p^3:
%    Kd = (3p - beta) / alpha
%    Kp = (3p^2 - gamma) / alpha
%    Ki = p^3 / alpha
%% ============================================================

% --- Scenario 1: Moderate bandwidth (~3x open-loop wn) ---
p1 = 3 * sqrt(gamma);
Kp1 = (3*p1^2 - gamma) / alpha;
Ki1 = p1^3 / alpha;
Kd1 = (3*p1 - beta) / alpha;
a1  = 0;          % feed-forward gain (start with 0)

% --- Scenario 2: Higher bandwidth (~5x open-loop wn) ---
p2 = 5 * sqrt(gamma);
Kp2 = (3*p2^2 - gamma) / alpha;
Ki2 = p2^3 / alpha;
Kd2 = (3*p2 - beta) / alpha;
a2  = 0;

% --- Scenario 3: Scenario 1 + feed-forward (a = Kp shifts zero left) ---
Kp3 = Kp1;
Ki3 = Ki1;
Kd3 = Kd1;
a3  = Kp1;        % a = Kp is a common choice: zero moves to -Ki/(2*Kp)

fprintf('\n=== Controller Gains ===\n');
fprintf('  [Scenario 1 - moderate, p=%.2f]  Kp=%.2f  Ki=%.2f  Kd=%.4f  a=%.2f\n', p1, Kp1, Ki1, Kd1, a1);
fprintf('  [Scenario 2 - fast,     p=%.2f]  Kp=%.2f  Ki=%.2f  Kd=%.4f  a=%.2f\n', p2, Kp2, Ki2, Kd2, a2);
fprintf('  [Scenario 3 - moderate + FF]       Kp=%.2f  Ki=%.2f  Kd=%.4f  a=%.2f\n', Kp3, Ki3, Kd3, a3);

%% ============================================================
%  BUILD CLOSED-LOOP TRANSFER FUNCTIONS
%  X(s)/Xref(s) = alpha*[(Kp+a)*s + Ki] /
%                 [s^3 + (beta+alpha*Kd)*s^2 + (alpha*Kp+gamma)*s + alpha*Ki]
%% ============================================================

function sys = build_cl_tf(alpha, beta, gamma, Kp, Ki, Kd, a)
    num = alpha * [Kp + a,  Ki];
    den = [1, ...
           beta + alpha*Kd, ...
           alpha*Kp + gamma, ...
           alpha*Ki];
    sys = tf(num, den);
end

sys1 = build_cl_tf(alpha, beta, gamma, Kp1, Ki1, Kd1, a1);
sys2 = build_cl_tf(alpha, beta, gamma, Kp2, Ki2, Kd2, a2);
sys3 = build_cl_tf(alpha, beta, gamma, Kp3, Ki3, Kd3, a3);

fprintf('\n=== Closed-Loop Poles ===\n');
fprintf('  Scenario 1: '); disp(pole(sys1).');
fprintf('  Scenario 2: '); disp(pole(sys2).');
fprintf('  Scenario 3: '); disp(pole(sys3).');

%% ============================================================
%  STEP RESPONSE — position command step to 0.05 m (5 cm)
%% ============================================================
t_end = 5;
t = linspace(0, t_end, 5000);
x_step = 0.05;    % 5 cm step command

[y1, t1] = step(x_step * sys1, t);
[y2, t2] = step(x_step * sys2, t);
[y3, t3] = step(x_step * sys3, t);

% Step info
info1 = stepinfo(sys1);
info2 = stepinfo(sys2);
info3 = stepinfo(sys3);

fprintf('\n=== Step Response Metrics (to unit step, scaled to 5cm) ===\n');
fprintf('  %-30s  %8s  %8s  %8s\n', 'Metric', 'Scen 1', 'Scen 2', 'Scen 3');
fprintf('  %-30s  %8.4f  %8.4f  %8.4f\n', 'Rise time (s)',         info1.RiseTime,    info2.RiseTime,    info3.RiseTime);
fprintf('  %-30s  %8.4f  %8.4f  %8.4f\n', 'Settling time (s)',     info1.SettlingTime,info2.SettlingTime,info3.SettlingTime);
fprintf('  %-30s  %8.2f  %8.2f  %8.2f\n', 'Overshoot (%)',         info1.Overshoot,   info2.Overshoot,   info3.Overshoot);
fprintf('  %-30s  %8.4f  %8.4f  %8.4f\n', 'Peak (m)',   x_step*info1.Peak, x_step*info2.Peak, x_step*info3.Peak);

%% ============================================================
%  PLOT 1: Step responses
%% ============================================================
figure('Name','Step Response','Position',[100 100 800 500]);
plot(t1, y1*x_step*1000, 'b-',  'LineWidth', 2); hold on;
plot(t2, y2*x_step*1000, 'r--', 'LineWidth', 2);
plot(t3, y3*x_step*1000, 'g-',  'LineWidth', 2);
yline(x_step*1000, 'k:', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Position (mm)');
title('Closed-Loop Step Response — Motor-Rack-Spring PIDF');
legend('Scenario 1: Moderate (3×wn)', ...
       'Scenario 2: Fast (5×wn)', ...
       'Scenario 3: Moderate + Feed-forward', ...
       'Reference (50 mm)', 'Location', 'southeast');
grid on; xlim([0 t_end]); ylim([-5 70]);

%% ============================================================
%  PLOT 2: Control effort (voltage) for Scenario 1
%  V(t) = Kp*e + Ki*integral(e) - Kd*xdot + a*xref
%  We simulate this manually using lsim + state-space
%% ============================================================

% Build closed-loop state-space for scenario 1 (easier to extract signals)
% States: [x1=x, x2=xdot, x3=integ(e)]
A_cl = [0,                    1,       0;
        -(alpha*Kp1 + gamma), -(beta + alpha*Kd1), alpha*Ki1;
        -1,                   0,       0];
B_cl = [0; alpha*(Kp1 + a1); 1];
C_x  = [1, 0, 0];       % position output
D_x  = [0];

% Simulate with step input
x0   = zeros(3,1);
r_in = x_step * ones(size(t));
sys_ss1 = ss(A_cl, B_cl, C_x, D_x);
[~, ~, x_states] = lsim(sys_ss1, r_in, t, x0);

x_pos  = x_states(:,1);
x_vel  = x_states(:,2);
x_integ= x_states(:,3);
e_t    = r_in.' - x_pos;

V_t = Kp1*e_t + Ki1*x_integ - Kd1*x_vel + a1*r_in.';

figure('Name','Control Effort','Position',[100 650 800 400]);
yyaxis left
plot(t, x_pos*1000, 'b-', 'LineWidth', 2);
yline(x_step*1000, 'b:', 'LineWidth', 1);
ylabel('Position (mm)');
yyaxis right
plot(t, V_t, 'r-', 'LineWidth', 1.5);
yline(12, 'r:', 'LineWidth', 1);
yline(-12,'r:', 'LineWidth', 1);
ylabel('Voltage V(t) [V]');
xlabel('Time (s)');
title('Scenario 1: Position and Control Voltage');
legend('x(t)', 'x_{ref}', 'V(t)', 'V_{max} = ±12V', 'Location','northeast');
grid on; xlim([0 t_end]);

%% ============================================================
%  PLOT 3: Spring constant sweep (k = 50, 100, 200, 500 N/m)
%% ============================================================
k_vals  = [50, 100, 200, 500];
colors  = {'b','r','g','m'};
figure('Name','Spring Constant Sweep','Position',[950 100 800 500]);
hold on;

for idx = 1:length(k_vals)
    kk = k_vals(idx);

    % Recompute plant coefficients for this k
    gamma_k = kk / M_eff;
    alpha_k = alpha;          % alpha doesn't depend on k
    beta_k  = beta;

    % Recompute gains for same relative bandwidth (3x wn for this k)
    p_k  = 3 * sqrt(gamma_k);
    Kp_k = (3*p_k^2 - gamma_k) / alpha_k;
    Ki_k = p_k^3 / alpha_k;
    Kd_k = (3*p_k - beta_k) / alpha_k;

    sys_k = build_cl_tf(alpha_k, beta_k, gamma_k, Kp_k, Ki_k, Kd_k, 0);
    [yk, tk] = step(x_step * sys_k, t);
    plot(tk, yk*1000, colors{idx}, 'LineWidth', 2, ...
         'DisplayName', sprintf('k = %d N/m', kk));
end

yline(x_step*1000, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Reference');
xlabel('Time (s)'); ylabel('Position (mm)');
title('Step Response vs Spring Constant k (gains re-tuned for each k)');
legend('Location','southeast'); grid on;
xlim([0 t_end]); ylim([-5 70]);

%% ============================================================
%  PLOT 4: Bode plot of open-loop plant G(s) = alpha/(s^2 + beta*s + gamma)
%% ============================================================
G_plant = tf(alpha, [1, beta, gamma]);

figure('Name','Bode - Open Loop Plant','Position',[950 650 800 400]);
bode(G_plant);
title('Open-Loop Plant G(s) Bode Plot (k = 100 N/m, no controller)');
grid on;

%% ============================================================
%  HELPER FUNCTION (must be at end of script in MATLAB)
%% ============================================================