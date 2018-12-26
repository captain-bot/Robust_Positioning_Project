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

% position = [0.666, 0.468, 0.244];            % Had good result for combined error
% ornt_quat = [0.206, -0.326, 0.818, -0.425];

% position = [0.6165, 0.0774, 0.4025];
% ornt_quat = [0.6838, 0.7173, 0.0799, -0.1064];

position = [1.082, 0.315, 0.189];
ornt_quat = [0.034, 0.833, 0.060, 0.548];
ee_trans = [ornt_quat position];
rotmat_check = compquat2rotmat(ornt_quat)
quat2rotm_matlab = quat2rotm(ornt_quat)
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

function [rotmat] = compquat2rotmat(q)
    rotmat = [q(1)^2 + q(2)^2-q(3)^2-q(4)^2, 2*q(2)*q(3)-2*q(1)*q(4), 2*q(2)*q(4)+2*q(1)*q(3);
        2*q(2)*q(3)+2*q(1)*q(4), q(1)^2-q(2)^2+q(3)^2-q(4)^2, 2*q(3)*q(4)-2*q(1)*q(2);
        2*q(2)*q(4)-2*q(1)*q(3), 2*q(3)*q(4)+2*q(1)*q(2), q(1)^2-q(2)^2-q(3)^2+q(4)^2];
    % Normalize the columns to 1
    for i = 1:3
        rotmat(1:3, i) = rotmat(1:3, i)/norm(rotmat(1:3, i));
    end
end