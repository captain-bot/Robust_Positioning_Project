clc;
clear;
close all;

% Success rate plots of robust and worst IKs for pre-grasp positioning task
% with cylindrical peg.
radii_range = 0.0270:0.0005:0.0325;
clearance = (0.072 - 2*radii_range)/2;
robust_success_cnt = [98.9, 98.5, 96.6, 95.8, 95.1, 92.2, 88.9, 84.3, 81.2, 77.4, 71.2, 63.9];
worst_success_count = [96.3, 95.5, 91.3, 90.0, 89.3, 85.1, 81.8, 77.0, 73.6, 66.3, 60.9, 54.2];

% Plot
plot(clearance, robust_success_cnt, '-o', 'MarkerFacecolor', 'b', 'Linewidth', 1.5);
hold on
plot(clearance, worst_success_count, '-o', 'MarkerFacecolor', 'r', 'Linewidth', 1.5);
grid on;
xlabel("radial clearance [m]")
ylabel("Success rate (%)")
title("Success rate with varying clearance")
legend("\Theta^*", "\Theta^-")
xlim([0.003, 0.0095]);
ylim([50, 105])

% Get the information of objective values for robust and worst IKs with
% varying joint error sigma values
sigval = 0.0025:0.0005:0.0050;
num_std = 3;
max_eig_robust = 0.5206;
max_eig_worst = 0.7510;

obj_robust_array = sqrt(max_eig_robust) * sigval * num_std;
obj_worst_array = sqrt(max_eig_worst) * sigval * num_std;

% Plot
figure(2)
plot(sigval, obj_robust_array, '-o', 'MarkerFacecolor', 'b', 'Linewidth', 1.5);
hold on;
plot(sigval, obj_worst_array, '-o', 'MarkerFacecolor', 'r', 'Linewidth', 1.5);
grid on;
xlabel("\sigma [rad]")
ylabel("max $\mathcal{\delta} x$", 'Interpreter', 'latex')
title("Change in objective value with varying \sigma")
legend("\Theta^*", "\Theta^-")