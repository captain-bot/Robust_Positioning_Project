clc;
clear;
close all;

% Test the accuracy of IK solutions computed using IK-Fast
ik_sol = [0.511934, 0.0804495, -0.922562, 1.77817, -2.08166, 0.584144, -2.61174];
trans_gripper = mycls.forward_kinematics(ik_sol, 'left_gripper');
disp(trans_gripper(:, :, 8));

% Test the accuracy of IK solutions computed using IK-Fast
robust_ik_sol = [-0.9660, -0.1012, 0.7830, 1.8130, -2.9563, 1.6396, 2.3091];
trans_gripper = mycls.forward_kinematics(robust_ik_sol, 'left_gripper');
disp(trans_gripper(:, :, 8));

% R = [-0.1616, 0.1788, 0.9705;
%     -0.0486, 0.9808, -0.1888;
%     -0.9857, -0.0777, -0.1498];
% pre_R = [cosd(8), -sind(8), 0; sind(8), cosd(8), 0; 0, 0, 1];
% resultant_R = pre_R * R;
% resultant_quat = rotm2quat(resultant_R)
