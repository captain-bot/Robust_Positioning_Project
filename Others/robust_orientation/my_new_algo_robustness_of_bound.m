clc
clear
close all

global ak; global alp; global d; global bax_base;

% Computed error bound in degree
inc_ang_bound = 3.34;

% Number of time you need to trial
num_trials = 1000;

% Define noise parameters
mu = 0; sigma = 0.025;

% Build Baxter Robot
bax_base = [0.7071   -0.7071         0    0.0640;    
            0.7071    0.7071         0    0.2590;
                 0         0    1.0000    0.1196;
                 0         0         0    1.0000];  % base transformations
rbt = build_baxter_robot();
ak = rbt(1, :);
alp = rbt(2, :);
d = rbt(3, :);

% IK solution
ik_sol = [0.2634,0.2297,-2.9647,0.2772,0.7893,-0.5446,-2.2652];
Tend = forward_kin_baxter(ik_sol);
qd = rotm2quat(Tend(1:3,1:3));

% Start simulation
success_count = 0;
for i = 1:num_trials
    % generate noise vector
    noise_vec = normrnd(mu,sigma,[1,length(ik_sol)]);
    ik_noise = ik_sol + noise_vec;
    
    % Compute forward kinematics with noisy IK solution
    Tend = forward_kin_baxter(ik_noise);
    qa = rotm2quat(Tend(1:3,1:3));
    
    % Compute included angle between desired and actual quaternion
    inc_ang = acos(qa*qd');
    if inc_ang > pi/2
        inc_ang = pi - inc_ang;
    end
    
    % Check if within bound
    inc_ang_deg = inc_ang*180/pi;
    if inc_ang_deg < inc_ang_bound
        success_count = success_count + 1;
    end
end
fprintf('Success: %2.6f\n', success_count);

% Solve forward kinematics for Baxter robot
function [Tf] = forward_kin_baxter(theta)
    global ak; global alp; global d; global bax_base;
    Tf = bax_base;
    for i = 1:length(ak)
        if i~=2
            th = theta(i);
        else
            th = theta(i) + pi/2;
        end
        Tf = Tf*lcl_trns(ak(i), alp(i), d(i), th);
    end
end