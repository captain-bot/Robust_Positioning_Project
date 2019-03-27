clc
clear
close all

% Joint solutions
robust_sol = [0.0052   -0.1660   -2.0927    1.1777    1.6105    2.0793    2.6467];
worst_sol = [-0.1565   -1.0611   -0.7174    1.1818    0.3113    1.5296    3.0430];

ang = robust_sol;
% ang = worst_sol;
% ang = [0.0052, 0, 0, 0, 0, 0, 0];

% % Solve forward kinematics of baxter robot
% frame_trans = mycls.forward_kinematics(ang, 'left_gripper');

baxter_l = bax_exp.const_robot('left_gripper');
baxter_l.fkine(ang);
baxter_l.plot(ang);
