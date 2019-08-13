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
        % gen_err = zeros(1, 7);
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

% Test case: 1
% pd = [0.6165; 0.077; 0.4025];
% qd = [0.6839, 0.7174, 0.0799, -0.1064];
% best_sol = [0.365997,-0.205692,-1.45802,1.66477,2.93037,-1.12361,-0.142083];
% worst_sol = [-0.15771,0.880958,-2.75321,1.71041,1.1743,1.69088,2.11322];

% Test case: 2
% pd = [0.626, 0.144, 0.234];
% qd = [0.6839, 0.7174, 0.0799, -0.1064];
% best_sol = [0.253749,-0.226852,-0.983314,1.48298,2.72877,-1.34349,0.284744];
% worst_sol = [0.318227,0.769608,-2.03301,1.42885,0.89078,1.58827,2.65648];

% Test case: 3
% pd = [0.710, 0.06, 0.21];
% qd = [0.6839, 0.7174, 0.0799, -0.1064];
% best_sol = [0.0349671,-0.0668138,-1.04091,1.17072,-0.350841,1.35373,-2.95109];
% worst_sol = [-0.614399,-0.441267,0.175757,1.24981,-1.43593,1.85213,-2.55588];

% Test case: 4
% pd = [0.701, 0.101, 0.380];
% qd = [0.6839, 0.7174, 0.0799, -0.1064];
% best_sol = [0.198838,0.15605,-1.72923,1.30662,-2.8767,-1.3215,-0.389751];
% worst_sol = [-0.303309,0.765257,-2.91418,1.34741,1.27177,1.81548,2.30824];

% Test case: 5
% pd = [0.776, 0.082, 0.273];
% qd = [0.6839, 0.7174, 0.0799, -0.1064];
% best_sol = [-0.0580828,-0.00642594,-1.24443,0.886486,2.9452,-1.47542,-0.0211799];
% worst_sol = [-0.539433,-0.394277,0.0771546,0.915658,-1.43098,1.85946,-2.84796];

% Test case: 6
% pd = [0.756, -0.013, 0.537];
% qd = [0.6839, 0.7174, 0.0799, -0.1064];
% best_sol = [-0.0768768,-0.0863806,-1.84751,1.02697,0.190167,1.31708,2.57386];
% worst_sol = [-0.431151,0.382093,-2.75107,1.12321,-1.96927,-1.55604,-0.925567];