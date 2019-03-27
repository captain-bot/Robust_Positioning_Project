clc
clear
close all

% Addpath to utilities folder
addpath('../utilities');

% Joint noise parameters
mu = 0;
num_std = 3;
sig = 0.010;

% DH parameter of 3R manipulator
ak = [0.10, 0.10, 0.05];
dk = [0, 0, 0];
alpk = [0, 0, 0];

% Parameters for random joint angles
max_th = 2*pi; min_th = 0;
max_num_example = 250;

th_array = [];
for num_example = 1:max_num_example
    th = (max_th - min_th).*rand(1, 3) + min_th;
    th_array = [th_array; th];

    % Solve forward kinematics
    T = zeros(4, 4, length(ak)+1);
    T(:, :, 1) = eye(4);
    for i = 1:length(ak)
        T(:, :, i+1) = T(:, :, i) * lcl_trns(ak(i), alpk(i), dk(i), th(i));
    end
    p_des = T(1:3, 4, end);

    % Compute spatial jacobian
    Tnew = T(:, :, 2:end);
    jacob = mycls.jacb_spatial(Tnew, T(:, :, 1));

    % Compute position error bound
    c = (num_std*sig)^2;
    sqrt_c = num_std*sig;
    [V, D] = eig(jacob(1:2, :)*jacob(1:2, :)');
    [max_eig, max_id] = max(diag(D));
    [min_eig, min_id] = min(diag(D));
    error_bound = sqrt_c*sqrt(max_eig);
    major_axis = error_bound;
    minor_axis = sqrt(c)*sqrt(min_eig);

    % Perform Monte-Carlo Simulation
    num_sample = 10000;
    mycount = 0;
    for j = 1:num_sample
        gen_err = normrnd(mu, sig, size(th));
        act_th = th + gen_err;
        act_T = zeros(4, 4, length(ak)+1);
        act_T(:, :, 1) = eye(4);
        for i = 1:length(ak)
            act_T(:, :, i+1) = act_T(:, :, i) * lcl_trns(ak(i), alpk(i), dk(i), act_th(i));
        end
        p_act = act_T(1:3, 4, end);
        eucledian_error = norm(p_des - p_act);

        if eucledian_error <= error_bound
            mycount = mycount + 1;
        end
    end
    fprintf("mycount: %d\n", mycount);
    fprintf('Success rate: %2.6f\n', mycount*100/num_sample);
end

for i = 1:3
    figure(i)
    edges = [0, pi/4, pi/2, 3*pi/4, pi, 5*pi/4, 6*pi/4, 7*pi/4, 2*pi];
    histogram(th_array(:, i), edges);
    grid on
    xlabel('\theta [rad]')
    ylabel('frequency')
end