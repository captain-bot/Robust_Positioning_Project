clc
clear;
close all;

% Desired joint solution
% solIKs = [0.2634, 0.2297, -2.9647, 0.2772, 0.7893, -0.5446, -2.2652];
% solIKs = [-1.18274,-0.914794,1.04918,1.19984,2.55429,-1.55604,0.469661];
% solIKs = [-0.8782, -0.4529, 0.1392, 0.7861, -0.1971, 0.7980, 0.1464];

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
Ll(7) = Link([0    0.229525+add_len     0     0     0    0], 'standard'); % start at joint w2 and move to end-effector

% Create the Robots Baxter_L(left arm) and Baxter_R(right arm)
Baxter_l = SerialLink(Ll, 'name', 'Baxter_L', 'base' , ...
                      transl(0.024645, 0.219645, 0.118588) * trotz(pi/4)...
                      * transl(0.055695, 0, 0.011038));

% joint limits
jl_min = [-1.7016 -2.147 -3.0541 -0.05 -3.059 -1.5707 -3.059];
jl_max = [1.7016 1.047 3.0541 2.618 3.059 2.094 3.059];

for num_config = 1:1%100
    % Generate random joint vector
    solIKs = zeros(1,length(jl_max));
%     for kk = 1:length(solIKs)
%         solIKs(kk) = (jl_max(kk)-jl_min(kk)).*rand(1,1) + jl_min(kk);
%     end
    solIKs = [0.4218   -1.0945   -0.4904    1.8765 2.1944   -1.0210   -2.5313];
%     solIKs = [-0.7857, -0.5288, 1.3509, 1.7874, -1.7541, 2.0061, 0.8417];
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
    inc_ang = acosd(q_star'*qd);
    fprintf('error bound: %2.6f deg\n',inc_ang);

    %////////////////////////////////////////////////////
    %   Monte-Carlo Simulation for performace check    //
    %////////////////////////////////////////////////////
    % Start simulation
    success_count = 0;
    num_trials = 10000;
    for i = 1:num_trials
        % generate noise vector
        noise_vec = normrnd(mu,sigma,[1,length(solIKs)]);
        joint_ang = solIKs + noise_vec;

        % Solve forward kinematics with noisy solutions
        forward_kin = Baxter_l.fkine(joint_ang);
        fk = [forward_kin.n,forward_kin.o,forward_kin.a,forward_kin.t];

        % Compute realized quaternion
        realized_quat = rotm2quat(fk(1:3,1:3))';
        cos_val = realized_quat'*qd;
        if cos_val < 0
            realized_quat = -realized_quat;
            cos_val = realized_quat'*qd;
        end

        % Determine success
        if acosd(cos_val) < inc_ang
            success_count = success_count + 1;
        end
    end
    fprintf('Success: %2.6f\n', success_count);
    fprintf('-------------------------------\n');
end

% Skew symmetric form of a QUATERNION
function [H] = compH(q)
    % Checked Ok! on Nov. 11th.
    H = [-q(2) q(1) q(4) -q(3);
         -q(3) -q(4) q(1) q(2);
         -q(4) q(3) -q(2) q(1)];
end