clc
clear
close all

% This code implements the new algorithm that I have thought out to counter
% the comments of the reviewers of RA-L.

addpath('../utilities');

% Generate joint solution
% added pi/2 with th2 to match with DH parameter of Baxter robot
solIKs = [1.5, -0.1+pi/2, 1.8, 2.0, 0.5, 0.5, 2.5];

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

% Solve forward kinematics
Tf = bax_base;
for i = 1:length(ak)
    Tf = Tf*lcl_trns(ak(i), alp(i), d(i), solIKs(i));
end

% Desired rotation matrix in end effector space
rot_mat = Tf(1:3, 1:3);

% Compute all the analytical Jacobians
allJacs = cmpAllJacs(solIKs, rbt, bax_base);

%/////////////////////////////////////////////////////////////////////%
%          Compute robust solution against rotation error             
% Solve Rotation Optimization for Each Inverse Kinematics Solution
%/////////////////////////////////////////////////////////////////////%
c = (num_std*sigma)^2;     % scaling factor comes from std of js error
quat_des = rotm2quat(rot_mat);
fprintf('Desired quaternion: \n'); disp(quat_des);

% Find the bases for tangential plane to quat_des
H_qd = newH(quat_des);

% Set algoritm parameters
res = 0.001;
tol = 1e-6;
radi_max = 0.2;

% Initialize variables
radi_min = 0;
old_included_ang = 0;
q_opt = NaN;
ang = 0:res:2*pi;
% Start Binary Search over radi values
while (radi_max-radi_min > tol)
    radi = (radi_max+radi_min)/2;    % always consider the median point
    gen_pt = zeros(3,length(ang));
    gen_pt(1,:) = radi*cos(ang);
    gen_pt(2,:) = radi*sin(ang);
    
    % Ellipsoid constraint flag
    ellp_sat = 0;

    % Transform  and represent them in thetangential plane of quat_des
    gen_pt = H_qd'*gen_pt;

    % Translate by amount of quat_des
    gen_pt = quat_des' + gen_pt;
    for j = 1:size(gen_pt,2)
        gen_pt(:,j) = gen_pt(:,j)/norm(gen_pt(:,j));
    end

    % Check ellipsoid constraint is satisfied
    for i = 1:size(gen_pt,2)
        func_val = (gen_pt(:,i) - quat_des')'*H_qd'*inv(allJacs(4:end,:)*allJacs(4:end,:)')*H_qd*(gen_pt(:,i) - quat_des');
        if (func_val - (c/4) < 1e-20)
            included_ang = acos(quat_des*gen_pt(:,i));
            % fprintf('radi: %2.6f\n',radi);
            % fprintf('included_ang: %2.6f\n',included_ang);            
            if included_ang > old_included_ang
                old_included_ang = included_ang;
                q_opt = gen_pt(:,i);
                r_opt = radi;
                radi_min = radi;
                ellp_sat = 1;
            end
            % fprintf('current included angle: %2.6f\n',old_included_ang);
            % fprintf('----------------------------\n');
            break;
        end
    end
    if ellp_sat == 0
        radi_max = radi;
    end
end

if ~isnan(q_opt)
    fprintf('Included angle(degree): %2.6f\n',included_ang*180/pi);
    fprintf('Maximizer (quaternion): \n'); disp(q_opt');
else
    fprintf('Could not reach optima. Try reducing res value\n');
end
