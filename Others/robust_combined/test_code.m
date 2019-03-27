clc
clear
close all

% Joint angles
% th = [-0.0619, -0.9151, -0.5675, 1.7231, 2.0478, -1.5560, 0.4143];
th = [0.218208, -0.938796, -0.11198, 1.78555, -1.08260, 1.834641, 1.170427];

% Solve forward kinematics and compute Jacobian
[frdkin, jac] = mycls.frdkin_jaco(th, "left_gripper");