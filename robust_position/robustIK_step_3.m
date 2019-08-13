clc
clear
close all

addpath("../utilities");

% Compute robust IK test
knum = 2;                       % number of standard deviation to consider
sigval = .0045;                 % standard deviation
cval = (knum*sigval)^2;         % scaling factor

elbow_up = true;                % consider only elbow-up solutions

v_dir = [0; 1; 0];              % direction of error minimization

[robust_sol, ik_array, Iindex, min_max_val] = mycls.cmpDirRobust('left_gripper_base', elbow_up, cval, v_dir);
% [robust_sol, ik_array, Iindex, min_max_val] = mycls.cmp_PRobust('left_gripper_base', elbow_up);

