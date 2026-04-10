% Christopher Wall u1467634
clear;
close all;
clc;

run1 = load("StepSize10.mat");

K = 2.5;
T = .168;

% Convert robot gains into V/rev
Kp = (490 * 0.0782480314961)
Ki = (92 * 0.0782480314961)
Kd = (36 * 0.0782480314961)

Gp = tf(K, [T, 1, 0]);
Gc = pid(Kp, Ki, Kd);
Gc1 = feedback(Gc*Gp, 1)

% ── Ramp trajectory parameters ───────────────────────────────────────────────
StepSize     = 10;    % rad — final desired position
ramp_end     = 1.5;   % s   — time at which ramp reaches StepSize
total_time   = 2.0;   % s   — total simulation duration (adjust as needed)
omega_ramp   = StepSize / ramp_end;   % rad/s during ramp (precomputed, matches Arduino)

% ── Build reference signal r(t) ───────────────────────────────────────────────
dt = 0.001;
t  = (0 : dt : total_time)';

r         = zeros(size(t));
ramp_idx  = t < ramp_end;
r(ramp_idx)  = omega_ramp .* t(ramp_idx);   % linear ramp phase
r(~ramp_idx) = StepSize;                     % hold phase

% ── Simulate closed-loop response to r(t) using lsim ─────────────────────────
y = lsim(Gc1, r, t);

% ── Performance metrics on simulated output ───────────────────────────────────
sim_final = y(end);

% Overshoot relative to StepSize (the commanded final position)
sim_peak_val = max(y);
sim_OS       = (sim_peak_val - StepSize) / StepSize * 100;
sim_peak_idx = find(y >= sim_peak_val * 0.9999, 1, 'first');
sim_peak_t   = t(sim_peak_idx);

% Settling time: last time output leaves 2% band around StepSize
band         = 0.02 * StepSize;
outside      = find(abs(y - StepSize) > band);
if isempty(outside)
    sim_ST = t(1);
else
    sim_ST = t(outside(end));
end
sim_ST_val = interp1(t, y, sim_ST);

% ── Experimental results ──────────────────────────────────────────────────────
exp_time  = run1.time;
exp_pos   = run1.position;
exp_final = mean(exp_pos(end-10:end));

exp_peak_val = max(exp_pos);
exp_peak_idx = find(exp_pos == exp_peak_val, 1, 'first');
exp_peak_t   = exp_time(exp_peak_idx);
exp_OS       = (exp_peak_val - exp_final) / exp_final * 100;

exp_band    = 0.02 * exp_final;
exp_outside = find(abs(exp_pos - exp_final) > exp_band);
if isempty(exp_outside)
    exp_ST = exp_time(1);
else
    exp_ST = exp_time(exp_outside(end));
end
exp_ST_val = interp1(exp_time, exp_pos, exp_ST);

% ── Plot ──────────────────────────────────────────────────────────────────────
figure();

plot(exp_time, exp_pos, 'r', 'Linestyle', '-', 'Marker', 'none', ...
    'DisplayName', 'Experimental');
hold on;

% Reference trajectory
plot(t, r, 'k--', 'DisplayName', 'Reference (Ramp)');
hold on;

% Simulated response
plot(t, y, 'b', 'DisplayName', 'Simulated');

% % Sim peak annotation
% plot(sim_peak_t, sim_peak_val, 'b^', 'MarkerSize', 10, 'MarkerFaceColor', 'auto', ...
%     'HandleVisibility', 'off');
% text(sim_peak_t, sim_peak_val, sprintf('  Sim OS: %.1f%%', sim_OS), ...
%     'FontSize', 9, 'VerticalAlignment', 'bottom');

% % Sim settling time annotation
% plot(sim_ST, sim_ST_val, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'auto', ...
%     'HandleVisibility', 'off');
% text(sim_ST, sim_ST_val, sprintf('  Sim ST: %.3fs', sim_ST), ...
%     'FontSize', 9, 'VerticalAlignment', 'top');
% 
% plot(exp_peak_t, exp_peak_val, 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'auto', 'HandleVisibility', 'off');
% text(exp_peak_t, exp_peak_val, sprintf('  Exp OS: %.1f%%', exp_OS),  'FontSize', 9, 'VerticalAlignment', 'bottom');
% 
% plot(exp_ST, exp_ST_val, 'rs', 'MarkerSize', 10, 'HandleVisibility', 'off');
% text(exp_ST, exp_ST_val, sprintf('  Exp ST: %.3fs', exp_ST),  'FontSize', 9, 'VerticalAlignment', 'top');

legend();
xlabel('Time [s]');
ylabel('Position [rad]');
title('Ramp Response: Simulated vs Experimental');
grid on;
