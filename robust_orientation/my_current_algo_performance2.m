clc;
clear;
close all;

robust_ik = [0.3135, -1.1787, -1.0018, 1.0451, -0.2102, 1.6396, -0.0047];
worst_ik = [-1.0411, -0.9403, 1.4856, 1.0361, -1.3185, 2.0793, 0.4996];
num_trials = 1000;

% Define noise parameters
mu = 0; sigma = 0.02; num_std = 3;

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

% Create the Robots Baxter_L(left arm)
Baxter_l = SerialLink(Ll, 'name', 'Baxter_L', 'base' , ...
                      transl(0.024645, 0.219645, 0.118588) * trotz(pi/4)...
                      * transl(0.055695, 0, 0.011038));

forward_kin = Baxter_l.fkine(worst_ik);
fk = [forward_kin.n,forward_kin.o,forward_kin.a,forward_kin.t];
desired_quat = rotm2quat(fk(1:3,1:3))';
success_count = 0;
success_robust = 0;
success_worst = 0;
for ii = 1:num_trials
    noise_vec = normrnd(mu,sigma,[1,length(robust_ik)]);
    robust_ik_noisy = robust_ik + noise_vec;
    worst_ik_noisy = worst_ik + noise_vec;
    
    %////////////////////////////////////////////////////
    % Solve forward kinematics with noisy solutions    //
    %///////////////////////////////////////////////////
    % Robust
    forward_kin = Baxter_l.fkine(robust_ik_noisy);
    fk = [forward_kin.n,forward_kin.o,forward_kin.a,forward_kin.t];
    robust_quat = rotm2quat(fk(1:3,1:3))';
    
    % Worst
    forward_kin = Baxter_l.fkine(worst_ik_noisy);
    fk = [forward_kin.n,forward_kin.o,forward_kin.a,forward_kin.t];
    worst_quat = rotm2quat(fk(1:3,1:3))';
    
    % compute dot product to determine correct sign
    dot_prod1 = desired_quat'*robust_quat;
    dot_prod2 = desired_quat'*worst_quat;
    
    if dot_prod1 < 0
        robust_quat = -robust_quat;
        dot_prod1 = desired_quat'*robust_quat;
    end
    
    if dot_prod2 < 0
        worst_quat = -worst_quat;
        dot_prod2 = desired_quat'*worst_quat;
    end

    if acosd(dot_prod1) < 2.99
        success_robust = success_robust + 1;
    end
    
    if acosd(dot_prod2) < 2.99
        success_worst = success_worst + 1;
    end
end
fprintf('Robust success count: %2.6f\n', success_robust);
fprintf('Worst success count: %2.6f\n', success_worst);