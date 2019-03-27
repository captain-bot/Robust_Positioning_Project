clc
clear;
close all;

% Define noise parameters
mu = 0; sigma = 0.010; num_std = 3;

% Left Arm
%             Theta d     a         alpha r/p  theta offset
add_len = 0.15;
Ll(1) = Link([0    0.27035  0.069  -pi/2  0    0], 'standard'); % start at joint s0 and move to joint s1
Ll(2) = Link([0    0        0       pi/2  0 pi/2], 'standard'); % start at joint s1 and move to joint e0
Ll(3) = Link([0    0.36435  0.0690 -pi/2  0    0], 'standard'); % start at joint e0 and move to joint e1
Ll(4) = Link([0    0        0       pi/2  0    0], 'standard'); % start at joint e1 and move to joint w0
Ll(5) = Link([0    0.37429  0.010  -pi/2  0    0], 'standard'); % start at joint w0 and move to joint w1
Ll(6) = Link([0    0        0       pi/2  0    0], 'standard'); % start at joint w1 and move to joint w2
Ll(7) = Link([0    0.229525+add_len 0     0    0    0], 'standard'); % start at joint w2 and move to end-effector

% Create the Robots Baxter_L(left arm) and Baxter_R(right arm)
Baxter_l = SerialLink(Ll, 'name', 'Baxter_L', 'base' , ...
                      transl(0.024645, 0.219645, 0.118588) * trotz(pi/4)...
                      * transl(0.055695, 0, 0.011038));

%///////////////////////////////////////////////////////
%            Compute Robust and Worst IKs             //
%///////////////////////////////////////////////////////
% Load all joint solutions
str_join = ["../utilities/data_files/left_ik", num2str(1), ".txt"];
str = join(str_join, "");
ik_array = load(str);
inc_ang_array = [];
for jsol_num = 1:size(ik_array,1)
    solIKs = ik_array(jsol_num,:);
    fprintf('Joint sol: '); disp(solIKs);

    % Solve forward kinematics
    forward_kin = Baxter_l.fkine(solIKs);
    fk = [forward_kin.n,forward_kin.o,forward_kin.a,forward_kin.t];

    % Compute Analytical Jacobian
    analytical_jacobian = Baxter_l.jacob0(solIKs);

    % //////////////////////////////////////////
    %   Start Rotation Optimization Algorithm //
    % //////////////////////////////////////////
    qd = rotm2quat(fk(1:3,1:3))';
    Hd = compH(qd);
    c_val = (num_std*sigma)^2;
    Jr = analytical_jacobian(4:end,:);
    [V,D] = eig(Jr*Jr');
    [max_eig, max_ind] = max(diag(D));
    Vmax = V(:,max_ind);
    v_vec = (1/2)*sqrt(c_val*max_eig)*Vmax;
    q_star = qd + Hd'*v_vec;
    q_star = q_star/norm(q_star);
%     inc_ang = acosd(q_star'*qd);
%     inc_ang_array = [inc_ang_array, inc_ang];

    % Newly added code
    rotmat_star = quat2rotm(q_star');
    inc_ang = acos(fk(1:3,3)'*rotmat_star(1:3,3));
    inc_ang_array = [inc_ang_array, inc_ang];
    % Newly added code end
    
    fprintf('error bound: %2.6f deg\n',inc_ang);
    fprintf('-----------------------------\n');
end
% Extract robust and worst IKs
[max_ang, maxid] = max(inc_ang_array);
[min_ang, minid] = min(inc_ang_array);
robust_ik = ik_array(minid,:);
worst_ik = ik_array(maxid,:);

% Skew symmetric form of a QUATERNION
function [H] = compH(q)
    % Checked Ok! on Nov. 11th.
    H = [-q(2) q(1) q(4) -q(3);
         -q(3) -q(4) q(1) q(2);
         -q(4) q(3) -q(2) q(1)];
end