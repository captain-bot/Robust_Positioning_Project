clc
clear
close all

% Addpath to utilities folder
addpath('../utilities');

% DH parameter of 3R manipulator
ak = [0.10, 0.10, 0.05];
dk = [0, 0, 0];  % add 0.15 to match left_gripper transform
alpk = [0, 0, 0];
th = [1.5, 1.5, 1.5];

% Solve forward kinematics
T = zeros(4, 4, length(ak)+1);
T(:, :, 1) = eye(4);
for i = 1:length(ak)
    T(:, :, i+1) = T(:, :, i) * lcl_trns(ak(i), alpk(i), dk(i), th(i));
end
p_des = T(1:3, 4, end);

% Compute spatial jacobian
Tnew = T(:, :, 2:end);
jacob = jacb_spatial(Tnew);

% Compute position error bound
mu = 0;
num_std = 3;
sig = 0.010;
c = (num_std*sig)^2;
sqrt_c = num_std*sig;
[V, D] = eig(jacob(1:2, :)*jacob(1:2, :)');
[max_eig, max_id] = max(diag(D));
[min_eig, min_id] = min(diag(D));
error_bound = sqrt_c*sqrt(max_eig);
major_axis = error_bound;
minor_axis = sqrt(c)*sqrt(min_eig);
alpha = atan2(V(2, max_id), V(1, max_id));
if alpha < 0
    alpha = pi + alpha;
end
fprintf("ellipse is rotated by %2.6f degrees\n", alpha*(180/pi));

% Perform Monte-Carlo Simulation
num_sample = 10000;
outlier_count = 0;
mycount = 0;
xcoord = zeros(1, num_sample);
ycoord = zeros(1, num_sample);
for j = 1:num_sample
    gen_err = normrnd(mu, sig, size(th));
    act_th = th + gen_err;
    act_T = zeros(4, 4, length(ak)+1);
    act_T(:, :, 1) = eye(4);
    for i = 1:length(ak)
        act_T(:, :, i+1) = act_T(:, :, i) * lcl_trns(ak(i), alpk(i), dk(i), act_th(i));
    end
    p_act = act_T(1:3, 4, end);
    xcoord(1, j) = p_act(1, 1);
    ycoord(1, j) = p_act(2, 1);
    eucledian_error = norm(p_des - p_act);
    
    in_ellip_val = ((cos(alpha)*(p_act(1) - p_des(1)) + sin(alpha)*(p_act(2) - p_des(2)))/major_axis)^2 ...
        + ((sin(alpha)*(p_act(1) - p_des(1)) + cos(alpha)*(p_act(2) - p_des(2)))/minor_axis)^2;
    
    if in_ellip_val <= 1
        mycount = mycount + 1;
    end
end
fprintf("mycount: %d\n", mycount);
fprintf('Success rate: %2.6f\n', mycount*100/num_sample);

c = linspace(1, 10, length(xcoord));
sz = 10;
scatter(xcoord,ycoord, sz, c, 'filled');
xlabel('x [m]');
ylabel('y [m]');
title('Position Error Bound');
grid on;
hold on;

% Draw bounding ellipsoid
ellp_coord = zeros(2, 100);
discretize_ang = linspace(0, 2*pi, length(ellp_coord));
for k = 1:length(ellp_coord)
    ellp_coord(1, k) = minor_axis*cos(discretize_ang(k));
    ellp_coord(2, k) = major_axis*sin(discretize_ang(k));
end
ellp_coord = V*ellp_coord + repmat(p_des(1:2), 1, length(ellp_coord));

plot(ellp_coord(1, :), ellp_coord(2, :));
plot(p_des(1), p_des(2), '*');
