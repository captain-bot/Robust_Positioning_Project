clc
clear
close all

addpath('../utilities');

% Provide end effector configuration and hovering distance
hov = 0;
%position = [0.2700, 0.4446, -0.1250];
%ornt_quat = [0.0834, -0.0939, -0.9878, 0.0918];
% position = [0.690,0.350,0.505];
% ornt_quat = [0.136163,-0.06605,0.9551,-0.254525];

% position = [0.2700, 0.4446, 0.250];
% ornt_quat = [0.0834, -0.0939, -0.9878, 0.0918];

position = [0.666, 0.468, 0.244];
ornt_quat = [0.206, -0.326, 0.818, -0.425];
ee_trans = [ornt_quat position];

% Write it in a file for cross validation later
fileID1 = fopen('../utilities/data_files/tool_frame_config.txt', 'w');
fprintf(fileID1,'%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f\n', ee_trans);

% Add hovering distance
ee_trans(1, 7) = ee_trans(1, 7) + hov;

% Get end effector transformation for IKFast
ik_fast_trns = mycls.IK_Fast_input_quat(ee_trans, 'left_gripper');
rot_ikfast_trns = ik_fast_trns(1:3, 1:3);
pos_ikfast_trns = ik_fast_trns(1:3, 4);
flatten_ikfast = [reshape(rot_ikfast_trns',1,9) pos_ikfast_trns'];

% Write it in a file
fileID = fopen('../utilities/data_files/se3file.txt', 'w');
nbytes = fprintf(fileID,'%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f\n', flatten_ikfast);