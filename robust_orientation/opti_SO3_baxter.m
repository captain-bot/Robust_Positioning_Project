clc
clear
close all

% Do not copy from this code. This is exclusively
% meant for baxter robot only which has different 
% forward kinematics parameters
%===================================================

addpath('../utilities');

% Generate joint solution
% added pi/2 with th2 to match with DH parameter of Baxter robot
solIKs = [1.0, -1.5+pi/2, 3.1, 1.5, 2.1, 1.5, 1.5];

% Define noise parameters
mu = 0; sigma = 0.025; num_std = 3;

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

% Solve forward kinematics
Tf = bax_base;
for i = 1:length(ak)
    Tf = Tf*lcl_trns(ak(i), alp(i), d(i), solIKs(i));
end

% Transformation
rot_mat = Tf(1:3, 1:3);

% Compute all the analytical Jacobians
allJacs = cmpAllJacs(solIKs, rbt, bax_base);

%/////////////////////////////////////////////////////////////////////%
%          Compute robust solution against rotation error             
% Solve Rotation Optimization for Each Inverse Kinematics Solution
%/////////////////////////////////////////////////////////////////////%
% c = 4*1e-4;                   % scaling factor comes from std of js error
c = (3*0.025)^2;
quat_des = [0 Tf(1:3, 1)'];
fprintf('Desired direction: \n'); disp(quat_des);

H_qd = funcH(quat_des);
a_vec = H_qd*quat_des';
k_val = quat_des(4)/quat_des(3);

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
    fprintf('robust quaternion: \n'); disp(wrstquat');
    fprintf('included angle: %6.2f deg\n', acosd(quat_des*wrstquat)); 
    fprintf('---------------------------------------------\n');
end
