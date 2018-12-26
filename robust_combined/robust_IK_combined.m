clc;
clear;
close all;
addpath('../utilities');

% Define joint error parameters
knum = 3;
sig = 0.0045;
cval = (knum*sig)^2;

% Define peg length
peg_len = 0.1;

% Get desired position and orientation
desired_config = load('../utilities/data_files/tool_frame_config.txt', 'w');
R_des = quat2rotm(desired_config(1,1:4));
P_des = desired_config(1,5:end);

% Load IK solutions
str_join = ["../utilities/data_files/left_ik", num2str(1), ".txt"];
str = join(str_join, "");
ik_array = load(str);

% Check if you are loading correct solution file

%//////////////////////////////////////////
%       RobustIK for POSE error          //
%//////////////////////////////////////////
error_measure_array = zeros(1,size(ik_array,1));
for ii = 1:size(ik_array, 1)
    tool_trns = mycls.forward_kinematics(ik_array(ii, :), 'left_gripper');       % Solve Forward Kinematics
%     thval = [-1.27761,0.627382,2.15623,1.34847,0.153395,-0.881736,0.528056];
%     tool_trns = mycls.forward_kinematics(thval, 'left_gripper');
    analytical_jac = mycls.jacb_spatial(tool_trns(:,:,2:end),tool_trns(:,:,1));  % Compute Analytical Jacobian
    Jp = analytical_jac(1:3,:);  % Position Jacobian
    Jr = analytical_jac(4:6,:);  % Rotation Jacobian
    
    %//////////////////////////////////
    %// Compute Position Error Bound //
    %//////////////////////////////////
    [~, Dp] = eig(Jp*Jp');
    max_eig = max(diag(Dp));
    worst_position_err = sqrt(cval*max_eig);
    
    %//////////////////////////////////
    %// Compute Rotation Error Bound //
    %//////////////////////////////////
    qd = desired_config(1,1:4)';
    Hd = compH(qd);
    [Vr,Dr] = eig(Jr*Jr');
    [max_eig, max_ind] = max(diag(Dr));
    Vmax = Vr(:,max_ind);
    v_vec = (1/2)*sqrt(cval*max_eig)*Vmax;
    q_star = qd + Hd'*v_vec;
    q_star = q_star/norm(q_star);
    worst_rotation_error = acosd(q_star'*qd);
    worst_rotmat = quat2rotm(q_star');
    
    %//////////////////////////////////
    %//      Weigh the errros        //
    %//////////////////////////////////
    error_measure_array(ii) = worst_position_err ...
        + peg_len*acos(worst_rotmat(1:3,3)'*R_des(1:3,3));
end

% Compute maximum and minimum error measures
[max_err, maxid] = max(error_measure_array);
[min_err, minid] = min(error_measure_array);
robust_ik = ik_array(minid,:);
worst_ik = ik_array(maxid,:);

% Results for best and worst case
fprintf('Robust case: \n');
fprintf('Minimum error: %2.6f\n',min_err);
fprintf('Robust IK: '); disp(robust_ik);
fprintf('------------------\n');
fprintf('Worst case: \n');
fprintf('Maximum error: %2.6f\n',max_err);
fprintf('Worst IK: '); disp(worst_ik);
fprintf('------------------\n');