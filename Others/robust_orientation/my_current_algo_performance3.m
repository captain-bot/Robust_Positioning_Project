clc;
clear;
close all;

% robust_ik = [1.0363, 0.1455, -1.1386, 2.2603, -1.4137, -1.4241, 2.8591];
% worst_ik = [-0.1450, -0.8088, 0.0974, 2.4054, 2.8304, -0.2367,-1.9206];
% robust_ik = [-1.2054   -0.3818    2.0583    2.1410   -0.7486    1.8375    2.1072];
% worst_ik = [1.6661   -1.3817   -1.9835    2.0793   -3.0390   -1.3801   -2.5432];
robust_ik = [0.8196   -0.7881   -1.0732    1.7786   -0.4208    0.8407 0.2448];
worst_ik = [0.8638    0.3713   -1.9451    1.8027   -1.8912   -1.2262 2.3387];
num_trials = 1000;

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

% Create the Robots Baxter_L(left arm)
Baxter_l = SerialLink(Ll, 'name', 'Baxter_L', 'base' , ...
                      transl(0.024645, 0.219645, 0.118588) * trotz(pi/4)...
                      * transl(0.055695, 0, 0.011038));

forward_kin = Baxter_l.fkine(worst_ik);
fk_des = [forward_kin.n,forward_kin.o,forward_kin.a,forward_kin.t];
desired_quat = rotm2quat(fk_des(1:3,1:3))';
success_count = 0;
success_robust = 0;
success_worst = 0;
old_max_inc_robust = 0;
old_max_inc_worst = 0;
for ii = 1:num_trials
    noise_vec = normrnd(mu,sigma,[1,length(robust_ik)]);
    robust_ik_noisy = robust_ik + noise_vec;
    worst_ik_noisy = worst_ik + noise_vec;
    
    %////////////////////////////////////////////////////
    % Solve forward kinematics with noisy solutions    //
    %///////////////////////////////////////////////////
    % Robust
    forward_kin = Baxter_l.fkine(robust_ik_noisy);
    fk_robust = [forward_kin.n,forward_kin.o,forward_kin.a,forward_kin.t];
    
    % Worst
    forward_kin = Baxter_l.fkine(worst_ik_noisy);
    fk_worst = [forward_kin.n,forward_kin.o,forward_kin.a,forward_kin.t];
    
    % Included angle between Z axis
    inc_angle_robust = acos(fk_des(1:3,3)'*fk_robust(1:3,3));
    inc_angle_worst = acos(fk_des(1:3,3)'*fk_worst(1:3,3));
    
    if old_max_inc_robust < inc_angle_robust
        old_max_inc_robust = inc_angle_robust;
    end
    
    if old_max_inc_worst < inc_angle_worst
        old_max_inc_worst = inc_angle_worst;
    end
    
    % Print angles
%     fprintf('Included agle robust case (rad): %2.6f\n', inc_angle_robust);
%     fprintf('Included agle worst case (rad): %2.6f\n', inc_angle_worst);
    
    if inc_angle_robust < inc_angle_worst
        success_count = 1 + success_count;
    end
    
%     if dot_prod1 < 0
%         robust_quat = -robust_quat;
%         dot_prod1 = desired_quat'*robust_quat;
%     end
%     
%     if dot_prod2 < 0
%         worst_quat = -worst_quat;
%         dot_prod2 = desired_quat'*worst_quat;
%     end
%     
%     if acosd(dot_prod1) < acosd(dot_prod2)
%         success_count = success_count + 1;
%     end
% 
%     if acosd(dot_prod1) < 1.06
%         success_robust = success_robust + 1;
%     end
%     
%     if acosd(dot_prod2) < 1.06
%         success_worst = success_worst + 1;
%     end
end
fprintf('Success count: %2.6f\n', success_count);
fprintf('Robust success count: %2.6f\n', success_robust);
fprintf('Worst success count: %2.6f\n', success_worst);
