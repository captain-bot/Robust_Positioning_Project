clc
clear
close all

% Plotting the statistical results
success_rob = [95.60, 93.93, 91.26, 89.39, 85.47, 80.80, 76.07, 69.50];
success_wor = [90.30, 87.70, 84.57, 82.06, 76.83, 71.79, 66.57, 60.80];
block_sz = 2*[0.029, 0.0295, 0.0300, 0.0305, 0.0310, 0.0315, 0.0320, 0.0325];

plot(block_sz, success_rob, 'b-', block_sz, success_wor, '-r', block_sz, success_rob, 'bo', block_sz, success_wor, 'ro');
xlim([0.057, 0.066])
xlabel('Block width [m]')
ylabel('Success rate [%]')
title('Effect of Block Size on Success Rate')
grid on
legend('Robust sol', 'Worst sol')