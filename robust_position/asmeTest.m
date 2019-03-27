clc
clear
close all

addpath('../utilities');

% Compute robust IK test
knum = 2;                       % number of standard deviation to consider
mu = 0;
sigval = 0.0045;                % standard deviation
cval = (knum*sigval)^2;         % scaling factor
peg_len = 0.100;                % length of the peg
error_tol_array = 0.003:0.001:0.010;
sample_num = 1000;

best_sol = [0.365997, -0.205692, -1.45802, 1.66477, 2.93037, -1.12361, -0.142083];
worst_sol = [-0.15771, 0.880958, -2.75321, 1.71041, 1.1743, 1.69088, 2.11322];

% Desired end-effector pose
pd = [0.6165; 0.077; 0.4025];
qd = [0.6839, 0.7174, 0.0799, -0.1064];
Rd = quat2rotm(qd);
sz_sol = length(best_sol);


success_best_array = [];
success_worst_array = [];
for error_tol = error_tol_array
    success_best = 0;
    success_worst = 0;
    for i = 1:sample_num
        gen_err = normrnd(mu, sigval, size(sz_sol));
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
plot(error_tol_array*1000, success_best_array, 'o-', 'MarkerFaceColor', 'b');
grid on
legend('success rate')
xlabel('clearance [mm]')
ylabel('Percentage of Success')