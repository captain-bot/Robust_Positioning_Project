clc
clear
close all

% This code is used to generate results of the second example in the
% journal. The example is about aligning x-axis of a frame attached at the
% tip of a spherical wrist to a desired direction.

addpath('../utilities');

% Direction of X-axis of rotation matrix
x_dir = [0.8296, 0.3322, -0.4488];
solIKs = IK_spherical_wrist(x_dir);

% Build robot
rbt = build_robot();
rbt_base = eye(4, 4);

% Compute all the analytical Jacobians
allJacs = cmpAllJacs(solIKs, rbt, rbt_base);

%/////////////////////////////////////////////////////////////////////%
%          Compute robust solution against rotation error             
% Solve Rotation Optimization for Each Inverse Kinematics Solution
%/////////////////////////////////////////////////////////////////////%
mu = 0; sig = 0.015; num_sig = 3;
c = (num_sig * sig)^2;
quat_des = [0 x_dir];      % Desired End Effector Rotation
H_qd = funcH(quat_des);
a_vec = H_qd*quat_des';
k_val = quat_des(4)/quat_des(3);
bound_array = [];
wrst_quat_array = [];

for i = 1:size(solIKs, 1)    
    % Compute Jacobian
    manp_jac = allJacs(:, :, i);   
    
    % Compute Rotation Error Ellipsoid Matrix
    rotErrMat = (4/c)*inv(manp_jac(4:6, :)*manp_jac(4:6, :)');
    
    % Compute Cholesky Decomposition
    L_err = chol(rotErrMat, 'lower');
    
    % Set alpha_max, alpha_min and Objective Value
    alpha_max = pi;
    alpha_min = pi/2;
    max_obj = 0;
    diff_angle = [];
    cfang_array = [];
    crad_array = [];
    
    ii=0;
    % Binary Search Over Range of Alphas
    while alpha_max - alpha_min > 1e-10
        alpha_avg = (alpha_max + alpha_min) / 2;
        
        if(ii==0)
            alpha_avg = alpha_min;
            ii=ii+1;
        end
       
        cone_axis = -a_vec;
        cone_hlf_ang = pi - alpha_avg;
        cfang_array = [cfang_array cone_hlf_ang];
        
        % compute cone radius (take any sign +/-)
        cone_ht = 2*cos(alpha_avg)^2*norm(a_vec);
        cone_rad = abs(2*norm(a_vec)*cos(alpha_avg)*sin(alpha_avg));
        crad_array = [crad_array cone_rad];
        
        [ellp_sat, obj, v_vec] = v_check_dir(cone_rad, cone_axis, cone_ht, L_err, k_val, H_qd);
        
        if ellp_sat == 1
            alpha_min = alpha_avg;
            max_obj = obj;
            diff_angle = [diff_angle acosd(obj)];
            opt_vvec = v_vec;
        else
            alpha_max = alpha_avg;
        end
    end
    
    if isempty(diff_angle) == 1        
        angle = {'no'};
    else
        angle = {diff_angle};
        worst_quat = {quat_des' + H_qd'*opt_vvec};
        wrstquat = quat_des' + H_qd'*opt_vvec;
        wrstang = diff_angle(1, end);
    end
    fprintf('joint angles used: \n'); disp(solIKs(i, :));
    fprintf('bounding quaternion: \n'); disp(wrstquat');
    fprintf('included angle: \n'); disp(acosd(quat_des*wrstquat));
    wrst_quat_array = [wrst_quat_array; wrstquat'];
    bound_array = [bound_array acosd(quat_des*wrstquat)];
    fprintf('---------------------------------------------\n')
end

[M_min, I_min] = min(bound_array);
best_bounding_quat = wrst_quat_array(I_min, :); 
[M_max, I_max] = max(bound_array);
worst_bounding_quat = wrst_quat_array(I_max, :);

fprintf('Info of Robust solution: \n');
fprintf('Joint sol: \n'); disp(solIKs(I_min, :));
fprintf('Bounding quaternion: \n'); disp(best_bounding_quat);
fprintf('Included angle(degree): \n'); disp(bound_array(1, I_min));
fprintf('=============================\n');

fprintf('Info of Worst solution: \n');
fprintf('Joint sol: \n'); disp(solIKs(I_max, :));
fprintf('Bounding quaternion: \n'); disp(worst_bounding_quat);
fprintf('Included angle(degree): \n'); disp(bound_array(1, I_max));
fprintf('=============================\n');

fprintf('Noise info: \n');
fprintf('mu: %2.6f\nsig: %2.6f\nnum_std: %d\n', mu, sig, num_sig);
fprintf('=============================\n');
