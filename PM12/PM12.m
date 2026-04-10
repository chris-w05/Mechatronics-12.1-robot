% Christopher Wall u1467634
clear;
close all;
clc;
run1 = load("P40_I300_D0_5.mat");
K = 22.5;
T = .168;

% (5cm / 2.54 cm/in) * 1.5 (gearbox ratio)
rad_to_in = (5/ 2.54) * 1.5;


% Convert robot gains into V/rev
Kp = (490/40)/ rad_to_in;
Ki = (92/40) / rad_to_in;
Kd = (36/40) / rad_to_in;

Gp = tf( K, [T, 1, 0]);
Gc = pid(Kp, Ki, Kd);

Gc1 = feedback(Gc*Gp, 1)
[y, t] = step(.8 * 10 / Kp * Gc1);

% found this cool function that makes finding OS / ST easy
stepinfo_sim = stepinfo(y, t, 'SettlingTimeThreshold', 0.02);
sim_final = y(end);
sim_OS = stepinfo_sim.Overshoot;
sim_ST = stepinfo_sim.SettlingTime;
sim_peak_val = sim_final * (1 + sim_OS/100);
sim_peak_idx = find(y >= sim_peak_val * 0.9999, 1, 'first');
sim_peak_t = t(sim_peak_idx);
sim_ST_val = interp1(t, y, sim_ST);

% Experimental results
exp_time = run1.time;
exp_pos  = run1.position;
exp_final = mean(exp_pos(end-10:end));
exp_peak_val = max(exp_pos);
exp_peak_idx = find(exp_pos == exp_peak_val, 1, 'first');
exp_peak_t   = exp_time(exp_peak_idx);
exp_OS = (exp_peak_val - exp_final) / exp_final * 100;

band = 0.02 * exp_final;
outside = find(abs(exp_pos - exp_final) > band);
if isempty(outside)
    exp_ST = exp_time(1);
else
    exp_ST = exp_time(outside(end));
end
exp_ST_val = interp1(exp_time, exp_pos, exp_ST);



figure();
% plot(exp_time, exp_pos, 'r', 'Linestyle', 'none', 'Marker', '*', ...
%     'DisplayName', 'Experimental');
% hold on;
plot(t, y, 'b', 'DisplayName', 'Simulated');
hold on;


plot(sim_peak_t, sim_peak_val, 'b^', 'MarkerSize', 10, 'MarkerFaceColor', 'auto', ...
    'HandleVisibility', 'off');
text(sim_peak_t, sim_peak_val,  sprintf('  Sim OS: %.1f%%', sim_OS), 'FontSize', 9, 'VerticalAlignment', 'bottom');


plot(sim_ST, sim_ST_val, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'auto', 'HandleVisibility', 'off');
text(sim_ST, sim_ST_val, sprintf('  Sim ST: %.3fs', sim_ST), 'FontSize', 9, 'VerticalAlignment', 'top');

% plot(exp_peak_t, exp_peak_val, 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'auto', 'HandleVisibility', 'off');
% text(exp_peak_t, exp_peak_val, sprintf('  Exp OS: %.1f%%', exp_OS),  'FontSize', 9, 'VerticalAlignment', 'bottom');
% 
% plot(exp_ST, exp_ST_val, 'rs', 'MarkerSize', 10, 'HandleVisibility', 'off');
% text(exp_ST, exp_ST_val, sprintf('  Exp ST: %.3fs', exp_ST),  'FontSize', 9, 'VerticalAlignment', 'top');

legend();
xlabel('Time [s]');
ylabel('Position [rad]');
title('Step Response: Simulated vs Experimental');
grid on;
