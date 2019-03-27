clc
clear
close all

addpath('../utilities');

% Compute robust IK test
knum = 2;                       % number of standard deviation to consider
sigval = .01;                   % standard deviation
cval = (knum*sigval)^2;         % scaling factor

elbow_up = true;                % consider only elbow-up solutions

v_dir = [0; 1; 0];              % direction of error minimization

th = [-1.6215, -0.5717, 1.3487, 1.5492, -0.9845, 1.3964, 0.5947];
% th = [-1.621, -0.571, 1.348, 1.549, -0.984, 1.396, 0.594];
transformation_mats = mycls.forward_kinematics(th, 'left_gripper');

% [robust_sol, ik_array, Iindex, min_max_val] = ...
%     mycls.cmpDirRobust('left_gripper', elbow_up, cval, v_dir);

[robust_sol, ik_array, Iindex, min_max_val] = ...
                             mycls.cmp_PRobust('left_gripper', elbow_up);