clc;
clear all;

rotation_matrix = [-0.526734   0.387384   0.756627; 0.794431   -0.092287   0.600302; 0.302374   0.917288   -0.259140];
position_vector = [1.061333;0.342267;0.214046];
flatten_ikfast = [reshape(rotation_matrix',1,9) position_vector'];
fileID = fopen('../utilities/data_files/se3file.txt', 'w');
nbytes = fprintf(fileID,'%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f\n', flatten_ikfast);