% This code is to determine which Euler angle convention to use
% when we are given with DH parameters of three intersecting joints
% of a spherical wrist
% Date: September 4th, 2018
%======================================================================

clc
clear
close all

% Build the robot
lnk = 0.050;
ak = [0 0 0];                          % DH aprameters of Spherical Joint
alpk = [-pi/2 pi/2 0];
dk = [0 0 lnk];
rbt = [ak; alpk; dk];

fprintf('|   a_k   |   alpha_k |  d_k  | theta_k |\n');
for ii = 1:length(ak)
    fprintf('|%2.6f | %2.6f | %2.6f| th%d|\n', rbt(1, ii), rbt(2, ii), rbt(3, ii), ii);
end
fprintf('\n');

% Generate desired quaternion
quat_vec = rand(1,3);
u1 = quat_vec(1); u2 = quat_vec(2); u3 = quat_vec(3);
quat_des = [sqrt(1-u1)*sin(2*pi*u2) sqrt(1-u1)*cos(2*pi*u2) sqrt(u1)*sin(2*pi*u3) sqrt(u1)*cos(2*pi*u3)];
rotmat_des = quat2rotm(quat_des);
fprintf('Randomly generated desired quat: \n');
disp(quat_des);
fprintf('In rotation matrix form: \n');
disp(rotmat_des);

% quat_des = [0.2893    0.3364    0.7108    0.5459];
% quat_des = [0 0 1 0];

% Joint solution
theta = quat2eul(quat_des, 'ZYZ');
fprintf("Forward kinematics will be solved for the following joint angles: \n"); disp(theta);

% Solve forward kinematics
Tf = eye(4);
for i = 1:length(ak)
    Tf = Tf*lcl_trns(ak(1, i), alpk(1, i), dk(1, i), theta(1, i));
end

% Check if you get back the same desired quaternion
fprintf('Returned quat: \n');
disp(rotm2quat(Tf(1:3, 1:3)));
fprintf('In rotation matrix form: \n');
disp(Tf(1:3, 1:3));


% Computes local transformations
function [T] = lcl_trns(a, alp, d, ang)
    T = [cos(ang) -sin(ang)*cos(alp) sin(ang)*sin(alp)  a*cos(ang);
         sin(ang)  cos(ang)*cos(alp) -cos(ang)*sin(alp) a*sin(ang);
                0           sin(alp)           cos(alp)          d;
                0                  0                  0          1];
end