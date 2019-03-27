clc
clear
close all

% Add required paths to MATLAB's search path
addpath('../utilities');

% Joint solution
solIKs = [1.0, -0.9, 1.0, 2.5, 1.5, 2.5, 1.5];

% Define noise parameters
mu = 0; sigma = 0.02; num_std = 3;

% Build Baxter Robot
% base transformations
bax_base = [0.7071   -0.7071         0    0.0640;
            0.7071    0.7071         0    0.2590;
                 0         0    1.0000    0.1196;
                 0         0         0    1.0000];

rbt = build_baxter_robot();
ak = rbt(1, :);
alp = rbt(2, :);
d = rbt(3, :);

% joint limits
jl_min = [-1.7016 -2.147 -3.0541 -0.05 -3.059 -1.5707 -3.059];
jl_max = [1.7016 1.047 3.0541 2.618 3.059 2.094 3.059];

% Solve forward kinematics for Baxter robot
Tf = bax_base;
for i = 1:length(ak)
    if i~=2
        th = solIKs(i);
    else
        th = solIKs(i) + pi/2;
    end        
    Tf = Tf*lcl_trns(ak(i), alp(i), d(i), th);
end

% Desired rotation matrix in end effector space
rot_mat = Tf(1:3, 1:3);

% Compute all the analytical Jacobians
allJacs = cmpAllJacs(solIKs, rbt, bax_base);

%///////////////////////////////////////////
%           Start of Optimization         //
%///////////////////////////////////////////
c = (num_std*sigma)^2;     % scaling factor comes from std of js error
quat_des = rotm2quat(rot_mat)';
fprintf('Desired quaternion: \n'); disp(quat_des);

% Construct H_qd
H_qd = newH(quat_des);

% Solve convex optimization problem
cvx_begin
    variable q(length(quat_des))
    minimize( q'*quat_des )
    subject to
%           q'*q <= 1
        sum(q) >= 1
        quat_des'*q >= 0
        (q-quat_des)'*(H_qd'*inv(allJacs(4:end,:)*allJacs(4:end,:)')*H_qd + 1e-6*eye(4,4))*(q-quat_des) <= c/4;
cvx_end

% Normalize the optimal quaternion to make it a unit quaternion
q = q/norm(q);

% Find the bound
ang_bound = acos(q'*quat_des);
fprintf('Bound from convex optimization(deg): %2.6f\n', ang_bound*180/pi);
