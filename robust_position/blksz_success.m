clc
clear
close all

% Plotting the statistical results
success_rob = [95.60, 93.93, 91.26, 89.39, 85.47, 80.80, 76.07, 69.50];
success_wor = [90.30, 87.70, 84.57, 82.06, 76.83, 71.79, 66.57, 60.80];
block_sz = 2*[0.029, 0.0295, 0.0300, 0.0305, 0.0310, 0.0315, 0.0320, 0.0325];
clearance = 0.072 - block_sz;

% Plot the result
plot(clearance, success_rob, 'o-', 'MarkerFaceColor', 'b');
hold on;
plot(clearance, success_wor, 'o-', 'MarkerFaceColor', 'r');
xlim([0.006, 0.016])
xlabel('Clearance [m]')
ylabel('Success rate [%]')
title('Effect of Block Size on Success Rate')
grid on
legend('\Theta^*', '\Theta^-')
