clc;
clear;
close all;

global quat_des; global allJacs; global H_qd; global coeff_confi;

% Add required paths to MATLAB's search path
addpath('../utilities');

% Fixed rotation matrix
% fixed_rotm = [0.0588   -0.8435    0.5339;
%              -0.2283   -0.5321   -0.8153;
%               0.9718   -0.0740   -0.2238];
fixed_rotm = [-0.7961   -0.5849    0.1554;
               0.5547   -0.8079   -0.1990;
               0.2419   -0.0723    0.9676];

% Define noise parameters
mu = 0; sigma = 0.015; num_std = 3;

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

% Load all joint solutions
str_join = ["../utilities/data_files/left_ik", num2str(1), ".txt"];
str = join(str_join, "");
ik_array = load(str);

wrst_ang = [NaN];
% jsols = randi(size(ik_array,1),1,5);
for jsol_num = 1:1%size(ik_array,1)
    % Joint solution
    % solIKs = ik_array(jsols(1,jsol_num),:);
    solIKs = ik_array(jsol_num,:);

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

    % Determine the premultiplying rotation matrix
    pre_rotm = fixed_rotm*rot_mat';

    %///////////////////////////////////////////
    %           Start of Optimization         //
    %///////////////////////////////////////////
    coeff_confi = (num_std*sigma)^2; % scaling factor comes from std of js error
    quat_des = rotm2quat(rot_mat)';
    fprintf('Desired quaternion: \n'); disp(quat_des);

    % Construct H_qd
    H_qd = newH(quat_des');

    % Construct H_qd_fixed
    quat_fixed = rotm2quat(fixed_rotm)';
    H_qd_fixed = newH(quat_fixed');

    % Solve convex optimization problem
    cvx_begin
        variable q(length(quat_des))
        minimize( q'*quat_fixed )
        subject to
           sum(q) >= 1
%             q'*q <= 1
            quat_fixed'*q >= 0
            (q-quat_fixed)'*(H_qd_fixed'*inv(pre_rotm*allJacs(4:end,:)*allJacs(4:end,:)'*pre_rotm')*H_qd_fixed + 1e-6*eye(4,4))*(q-quat_fixed) <= coeff_confi/4;
    cvx_end

    % Normalize the optimal quaternion to make it a unit quaternion
    q = q/norm(q);

    rot_computed = quat2rotm(q');
    rot_computed_actual_frame = pre_rotm'*rot_computed;
    quat = rotm2quat(rot_computed_actual_frame)';

    % Find the bound
    % ang_bound = acos(q'*quat_des);
    ang_bound = acos(q'*quat_fixed);
    fprintf('Bound from convex optimization(deg): %2.6f\n', ang_bound*180/pi);

    % Perform binary search over cos_th value to compute a tighter bound
    cos_th = quat'*quat_des;
    if cos_th < 0
        quat = -1*quat;
        cos_th = quat'*quat_des;
    end
    tic;
    wrst_ang(jsol_num) = check_ellipse_constraint(cos_th);
    toc;
    fprintf('===================================================\n\n\n\n');
end

%////////////////////////////////////////////////
%       Extract Robust and Worst IKs           //
%////////////////////////////////////////////////
[max_ang,max_idx] = max(wrst_ang);
[min_ang,min_idx] = min(wrst_ang);
robust_ik = ik_array(min_idx,:);
worst_ik = ik_array(max_idx,:);

fprintf('Robust IK: '); disp(robust_ik);
fprintf('Robust included angle(deg): '); disp(min_ang);

fprintf('Worst IK: '); disp(worst_ik);
fprintf('Worst included angle(deg): '); disp(max_ang);

