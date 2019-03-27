clc
clear
close all

global ak; global alp; global d; global bax_base;

% Number of time you need to trial
num_trials = 1000;

% Tolerable Error Margin
err_margin = 0.9;

% Performance test
% worst_ik = [0.1754   -0.1311    0.2831    0.0557    0.6596    0.5841    0.9024];
% robust_ik = [0.3188   -0.0949   -0.8140    0.0920   -1.3919   -0.5079   -2.2417];

% Works
% worst_ik = [0.2455,-0.1354,-0.3415,0.0688,1.2575,0.5548,0.9267];
% robust_ik = [0.1361,0.0997,2.3575,0.0955,1.8614,-0.5153,-2.4043];

robust_ik = [0.0578   -0.1344    0.9374    0.2150    0.0748    0.5109    0.8315];
worst_ik = [0.2701    0.1127   -2.7476    0.0588    0.6972   -0.4640   -2.4105];

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

% Get desired quaternion
Tend = forward_kin_baxter(robust_ik);
qd = rotm2quat(Tend(1:3,1:3));

% Start simulation
success_count = 0;
robust_success = 0;
worst_success = 0;
for i = 1:num_trials
    % generate noise vector
    noise_vec = normrnd(mu,sigma,[1,length(worst_ik)]);
    worst_ang = worst_ik + noise_vec;
    robust_ang = robust_ik + noise_vec;
    
    % Solve forward kinematics with noisy worst solutions
    worst_Tend = forward_kin_baxter(worst_ang);
    worst_quat = rotm2quat(worst_Tend(1:3,1:3));
    worst_cos_th = worst_quat*qd';
    if worst_cos_th < 0
        worst_quat = -worst_quat;
        worst_cos_th = worst_quat*qd';
    end
    
    % Solve forward kinematics with noisy robust solutions
    robust_Tend = forward_kin_baxter(robust_ang);
    robust_quat = rotm2quat(robust_Tend(1:3,1:3));
    robust_cos_th = robust_quat*qd';
    if robust_cos_th < 0
        robust_quat = -robust_quat;
        robust_cos_th = robust_quat*qd';
    end
    
    % Determine success
    if acosd(robust_cos_th) < acosd(worst_cos_th)
        success_count = success_count + 1;
    end
    if acosd(robust_cos_th) < err_margin
      robust_success = robust_success + 1;
    end
    if acosd(worst_cos_th) < err_margin
      worst_success = worst_success + 1;
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