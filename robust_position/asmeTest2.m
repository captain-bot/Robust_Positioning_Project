clc
clear
close all

addpath('../utilities');

% This code generates plots of success rates for Robust and Worst IK
% for Peg-In-Hole Problem considering both position and orientation 
% erorr of the left_gripper frame. The IK_{best} and IK_{worst} are
% generated using robust_ik_app_one_config.py code

% Compute robust IK test
knum = 2;                       % number of standard deviation to consider
mu = 0;
sigval = 0.0045;                % standard deviation
cval = (knum*sigval)^2;         % scaling factor
peg_len = 0.100;                % length of the peg
error_tol_array = 0.003:0.001:0.010;
sample_num = 1000;

pd = [0.776; 0.082; 0.273];
qd = [0.6839, 0.7174, 0.0799, -0.1064];

best_sol = [-0.0580828,-0.00642594,-1.24443,0.886486,2.9452,-1.47542,-0.0211799];
worst_sol = [-0.539433,-0.394277,0.0771546,0.915658,-1.43098,1.85946,-2.84796];

% Desired end-effector pose
Rd = quat2rotm(qd);
sz_sol = length(best_sol);
success_best_array = [];
success_worst_array = [];
for error_tol = error_tol_array
    success_best = 0;
    success_worst = 0;
    for i = 1:sample_num
        gen_err = normrnd(mu, sigval, size(sz_sol));
%         gen_err = zeros(1, 7);
        best_noise = best_sol + gen_err;
        worst_noise = worst_sol + gen_err;

        transformation_best = mycls.forward_kinematics(best_noise, 'left_gripper');
        transformation_worst = mycls.forward_kinematics(worst_noise, 'left_gripper');

        error_best = norm(transformation_best(1:3, 4, end) - pd) + peg_len*norm(transformation_best(1:3, 3, end) - Rd(1:3, 3)); 
        error_worst = norm(transformation_worst(1:3, 4, end) - pd) + peg_len*norm(transformation_worst(1:3, 3, end) - Rd(1:3, 3));

        if error_best < error_tol
            success_best = success_best + 1;
        end

        if error_worst < error_tol
            success_worst = success_worst + 1;
        end
    end
    fprintf('best success: %2.2f\n', success_best);
    fprintf('worst success: %2.2f\n', success_worst);
    fprintf('------------------------\n');
    
    success_best_array = [success_best_array, success_best*(100/sample_num)];
    success_worst_array = [success_worst_array, success_worst*(100/sample_num)];
end

% Plot the trend
plot(error_tol_array*1000, success_best_array, 'bo-', 'MarkerFaceColor', 'b', 'LineWidth', 2);
hold on;
grid on;
plot(error_tol_array*1000, success_worst_array, 'ro-', 'MarkerFaceColor', 'r', 'LineWidth', 2);
legend('\Theta_{best}', '\Theta_{worst}')
xlabel('clearance [mm]')
ylabel('Success rate [%]')
xlim([2, 11])