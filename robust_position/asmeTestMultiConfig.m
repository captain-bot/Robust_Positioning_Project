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

% Configurations are in [x, y, z, q0, q1, q2, q3] format
configs_collection = [0.6165, 0.077, 0.4025, 0.6839, 0.7174, 0.0799, -0.1064;
    0.626, 0.144, 0.234, 0.6839, 0.7174, 0.0799, -0.1064;
    0.710, 0.06, 0.21, 0.6839, 0.7174, 0.0799, -0.1064;
    0.701, 0.101, 0.380, 0.6839, 0.7174, 0.0799, -0.1064;
    0.776, 0.082, 0.273, 0.6839, 0.7174, 0.0799, -0.1064];

best_sols_collection = [0.365997,-0.205692,-1.45802,1.66477,2.93037,-1.12361,-0.142083;
    0.253749,-0.226852,-0.983314,1.48298,2.72877,-1.34349,0.284744;
    0.0349671,-0.0668138,-1.04091,1.17072,-0.350841,1.35373,-2.95109;
    0.198838,0.15605,-1.72923,1.30662,-2.8767,-1.3215,-0.389751;
    -0.0580828,-0.00642594,-1.24443,0.886486,2.9452,-1.47542,-0.0211799];

worst_sols_collection = [-0.15771,0.880958,-2.75321,1.71041,1.1743,1.69088,2.11322;
    0.318227,0.769608,-2.03301,1.42885,0.89078,1.58827,2.65648;
    -0.614399,-0.441267,0.175757,1.24981,-1.43593,1.85213,-2.55588;
    -0.303309,0.765257,-2.91418,1.34741,1.27177,1.81548,2.30824;
    -0.539433,-0.394277,0.0771546,0.915658,-1.43098,1.85946,-2.84796];

for num_case = 1:size(configs_collection, 1)
    Rd = quat2rotm(configs_collection(num_case, 4:end));
    pd = configs_collection(num_case, 1:3)';
    success_best_array = [];
    success_worst_array = [];
    best_sol = best_sols_collection(num_case, :);
    worst_sol = worst_sols_collection(num_case, :);
    sz_sol = length(best_sol);
    for error_tol = error_tol_array
        success_best = 0;
        success_worst = 0;
        for i = 1:sample_num
            gen_err = normrnd(mu, sigval, size(sz_sol));
%             gen_err = zeros(1, 7);
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
    fprintf("Case %d completed\n", num_case);
    fprintf("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n");
    if num_case == 1
        % Plot the trend
        plot(error_tol_array*1000, success_best_array, 'ro-', 'MarkerFaceColor', 'r', 'LineWidth', 2);
        hold on;
    elseif num_case == 2
        plot(error_tol_array*1000, success_best_array, 'go-', 'MarkerFaceColor', 'g', 'LineWidth', 2);
    elseif num_case == 3
        plot(error_tol_array*1000, success_best_array, 'bo-', 'MarkerFaceColor', 'b', 'LineWidth', 2);
    elseif num_case == 4
        plot(error_tol_array*1000, success_best_array, 'mo-', 'MarkerFaceColor', 'm', 'LineWidth', 2);
    elseif num_case == 5
        plot(error_tol_array*1000, success_best_array, 'co-', 'MarkerFaceColor', 'c', 'LineWidth', 2);
    end       
end

grid on
legend('configuration 1', 'configuration 2', 'configuration 3', 'configuration 4', 'configuration 5');
xlabel('clearance [mm]')
ylabel('Success rate [%]')
xlim([2, 11])