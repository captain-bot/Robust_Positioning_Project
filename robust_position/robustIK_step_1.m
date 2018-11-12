clc
clear
close all

addpath('../utilities');

% Provide end effector configuration and hovering distance
hov = 0;
position = [0.71305, 0.378638, 0.30];
ornt_quat = [0.008595, 0.999159, 0.0369768, 0.0154953];
ee_trans = [ornt_quat position];

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

