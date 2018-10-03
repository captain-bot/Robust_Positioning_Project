clc
clear
close all

% bt = 0.029;
bt = 0.029;
blk_ht = 2*bt;

% Left Gripper Plate vertices in its body frame
lp_vrtx = [0.020 0.002 -0.0175;
          -0.020 0.002 -0.0175;
          -0.020 0.002 0.0175;
           0.020 0.002 0.0175;
          -0.020 0 -0.0175;
          -0.020 0 0.0175;
          0.020 0 0.0175;
          0.020 0 -0.0175;];

% Right Gripper Plate vertices in its body frame
rp_vrtx = [0.020 0 -0.0175;
          -0.020 0 -0.0175;
          -0.020 0 0.0175;
           0.020 0 0.0175;
          -0.020 -0.002 -0.0175;
          -0.020 -0.002 0.0175;
          0.020 -0.002 0.0175;
          0.020 -0.002 -0.0175;];
 
% Block vertices in its body frame
blk_vrtx = [-bt bt blk_ht;
            bt bt blk_ht;
            bt bt 0;
            -bt bt 0;
            bt -bt blk_ht;
            bt -bt 0;
            -bt -bt 0;
            -bt -bt blk_ht];

 % Left Gripper Plate Normals
 lp_nrm = [-1 0 0;
            0 1 0;
            0 0 -1;
            1 0 0;
            0 -1 0;
            0 0 1];
        
 % Right Gripper Plate Normals
  rp_nrm = [-1 0 0;
            0 1 0;
            0 0 -1;
            1 0 0;
            0 -1 0;
            0 0 1];

 % Block Normals
 blk_nrm = [1 0 0;
            0 1 0;
            0 0 1;
           -1 0 0;
            0 -1 0;
            0 0 -1];
        
% save the data in a .mat file
save('coord_normals.mat', 'lp_vrtx', 'rp_vrtx', 'blk_vrtx', 'lp_nrm', 'rp_nrm', 'blk_nrm');