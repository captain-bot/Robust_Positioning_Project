clc
clear
close all

% DH parameter of 3R manipulator
% lnk = 0.050;
% ak = [0 0 0];                          
% alpk = [-pi/2 pi/2 0];
% dk = [0 0 lnk];
ak = [0.10, 0.10, 0.05];
dk = [0, 0, 0]; 
alpk = [0, 0, 0];
rbt = [ak; alpk; dk];
% th = [0.2, 0.15, 0.3];
% th = [-0.9533   -1.2483   -1.6456];
% th = [0.20,-0.15,-0.30];
% th = [1.5, 1.5, 1.5];
% th = [1.0, 0.5, 0.25];
th = [-0.50, 0.25, -1.50];

% Solve forwad kinematics using my functions
% T = zeros(4, 4, length(ak));
% T(:, :, 1) = lcl_trns(ak(1), alpk(1), dk(1), th(1));
% for i = 2:length(ak)
%     T(:, :, i) = T(:, :, i-1)*lcl_trns(ak(i), alpk(i), dk(i), th(i));
% end
% my_spatial_jac = jacb_spatial(T);
% fprintf('My functions output: \n');
% fprintf('Forward kinematics: \n'); disp(T(:, :, end));
% fprintf('Jacobian matrix: \n'); disp(my_spatial_jac);
% fprintf('----------------------\n');
% 
% j_val = cmpAllJacs(th, rbt);


% Construct robot
Ll(1) = Link ([0    dk(1)  ak(1)      alpk(1)  0    0]);     
Ll(2) = Link ([0    dk(2)  ak(2)      alpk(2)  0    0]);  
Ll(3) = Link ([0    dk(3)  ak(3)      alpk(3)  0    0]);

arm = SerialLink(Ll, 'name', 'planar_3R');

% Solve forward kinematics
fk = arm.fkine(th);
jaco_mat = arm.jacob0(th);

fprintf('Toolbox output: \n');
fprintf('Forward kinematics: \n'); disp(fk);
fprintf('Jacobian matrix: \n'); disp(jaco_mat);
