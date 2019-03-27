clc
clear
close all

addpath('../utilities');

num_of_simulation = 100;
success_array = zeros(1, num_of_simulation);

% Build robot
rbt = build_robot();
rbt_base = eye(4, 4);

for i = 1:num_of_simulation
    fprintf('Simulation no: %d\n', i);
    
    %/////////////////////////
    % Define input arguments//
    %/////////////////////////
    
    % Generate desired quaternion
    quat_vec = rand(1,3);
    u1 = quat_vec(1); u2 = quat_vec(2); u3 = quat_vec(3);
    quat_des = [sqrt(1-u1)*sin(2*pi*u2) sqrt(1-u1)*cos(2*pi*u2) sqrt(u1)*sin(2*pi*u3) sqrt(u1)*cos(2*pi*u3)];
    % quat_des = [0.2893 0.3364 0.7108 0.5459]; (in paper)
    fprintf('Desired quaternion: '); disp(quat_des);
    
    % Solve IK for desired quaternion
    sol = quat2eul(quat_des, 'ZYZ');
    fprintf('Joint solution: '); disp(sol);
    
    % Joint space uncertainty parameters
    mu = 0;
    sigma = 0.025;
    num_std = 3;
    
    fprintf('Joint-space uncertainty parameters: \n');
    fprintf('mu = %2.4f  ', mu);
    fprintf('sigma = %2.4f  ', sigma);
    fprintf('no of std = %d  \n\n', num_std);

    % Define parameters for Monte-Carlo simulation
    num_trial = 10000;

    % Call find_robust_quat
    [q_r, included_ang] = find_robust_quat(quat_des, sol, mu, sigma, num_std, rbt, rbt_base);

    % Perform Monte-Carlo simulation
    success_array(i) = MCSimulation(num_trial,mu, sigma, num_std, quat_des, q_r, sol, rbt, rbt_base);
end
