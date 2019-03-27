clc
clear all
close all

% Variation of sigma
sig_val = [0.0005, 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007,...
    0.008, 0.009, 0.01];

% Variation pf position errors at peg-tip
best_sol_err = [0.0012, 0.0025, 0.0051, 0.0077, 0.0102, 0.0128, 0.0154, 0.0180, 0.0205, 0.0231, 0.0257];
worst_sol_err = [0.0014, 0.0028, 0.0056, 0.0084, 0.0112, 0.0140, 0.0168, 0.0197, 0.0225, 0.0253, 0.0281];

% Plot the result
% plot(sig_val, best_sol_err, 'b');
% hold on;
% plot(sig_val, worst_sol_err, 'r');
plot(sig_val, best_sol_err, 'o-', 'MarkerFaceColor', 'b');
hold on
plot(sig_val, worst_sol_err, 'o-', 'MarkerFaceColor', 'r');
xlabel('\sigma [radian]')
ylabel('Position error at peg-tip [m]')
grid on
legend