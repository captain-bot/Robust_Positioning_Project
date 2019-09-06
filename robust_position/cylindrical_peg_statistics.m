clc
clear
close all

addpath ('../utilities');

% Define the global variables
global spawn_pos, global hc, global rc, global cyn_pt;
global base_T_lft, global base_T_rft, global base_lpN, global base_rpN;

% robust_success_cnt = [98.9, 98.5, 96.6, 95.8, 95.1, 92.2, 88.9, 84.3, 81.2, 77.4, 71.2, 63.9];
% worst_success_count = [96.3, 95.5, 91.3, 90.0, 89.3, 85.1, 81.8, 77.0, 73.6, 66.3, 60.9, 54.2];

% Transformation of the block with respect to base frame
ornt_peg = eye(3, 3);
% spawn_pos = [0.864; 0.419; 0.064-0.035]; % spawn pose of the peg
% spawn_pos = [1.070; 0.404; 0.240-(0.025)]; % spawn pose of the peg
% hc = 0.075; rc = 0.029;  % height and radius of the peg
% base_T_peg = [ornt_peg(1:3, 1:3) spawn_pos; zeros(1, 3) 1];

% spawn_pos = [0.713; 0.378; 0.300-0.040];
% hc = 0.075; rc = 0.030;  % height and radius of the peg
% Get the approach and grasp configuration joint space
% robust_sol = [0.0046, -0.1660, -2.0927, 1.1785, 1.6101, 2.0793, 2.6452];
% worst_sol = [-0.1556, -1.0607, -0.7204, 1.1824, 0.3127, 1.5296, 3.0410];

spawn_pos = [0.700; 0.350; 0.300-0.040];
hc = 0.075; rc = 0.0325;                    % height and radius of the peg
robust_sol = [0.0021, -0.1663, -2.0912, 1.2561, 1.5661, 2.0793, 2.5537];
worst_sol = [-0.2360, -1.1484, -0.6065, 1.2717, 0.2210, 1.4857, 3.0410];

base_T_peg = [ornt_peg(1:3, 1:3) spawn_pos; zeros(1, 3) 1];

% Select which solution to execute
% exe_sol = "robust";
exe_sol = "worst";

% Select total number of samples
total_sample = 1000;

% Decide Gaussian Noise Parameters
mu_val = 0; std_val = 0.0045;
lb = -100; ub = 100;

% Define the safe distance around the peg
safe_dist = 0.001;

%///////////////////////////////////////////////////////////////////////%
%///////////////////// DO NOT TOUCH ////////////////////////////////////%
%///////////////////////////////////////////////////////////////////////%
% Load data
data = load('coord_normals.mat');
fixed_T = load('fixed_trns.mat');

% Add noise with desired configuration
qang_r = robust_sol;

% Generate points that represent the cylinder
[X,Y,Z] = cylinder(rc);
Z = hc * Z;
X_new = base_T_peg(1:3, 1:3) * [reshape(X, [1, 42]); reshape(Y, [1, 42]);  reshape(Z, [1, 42])] + base_T_peg(1:3, 4);
cyn_pt = X_new(:, 10); % chose the 10th point

if exe_sol == "robust"
    sol = robust_sol;
else
    sol = worst_sol;
end

radii_range = 0.0270:0.0005:0.0325;
success_cnt_array = zeros(1, length(radii_range));
itr = 0;
for rc = radii_range
    success_cnt = 0;
    for num_run = 1:total_sample
        % Generate gaussian noise. Input order: (mean, std, lb, ub, sz)
        gn = mycls.gauss_noise(mu_val, std_val, lb, ub, 7);

        % Add noise with desired configuration
        qang = sol + gn;
        %     qang = worst_sol + gn;

        % Check collision with robust and worst solutions
        [base_lp_vertx, base_rp_vertx, min_l, min_r] = check_success(qang, fixed_T, data);

        if min_l > safe_dist && min_r > safe_dist
            success_cnt = success_cnt + 1;
        end
    end
    fprintf("Success rate: %2.3f\n", success_cnt*100/total_sample);
    itr = itr + 1;
    success_cnt_array(itr) = success_cnt;
end

% % Plot 3D shapes
figure(1);
box3d(base_lp_vertx, 2);
box3d(base_rp_vertx, 2);

% Plot the cylindrical peg
surf(reshape(X_new(1, :), [2, 21]), reshape(X_new(2, :), [2, 21]), reshape(X_new(3, :), [2, 21]), 'FaceColor', [0.1, 0.1, 0.3], 'Facealpha', 0.7, 'EdgeColor', 'none');
grid on;

% Plot the frames
% Draw frame for the peg
draw_frame(base_T_peg(1:3, 1:3), base_T_peg(1:3, 4));
% Draw frame for the left gripper plate
R_lp = [base_lpN(:, 4), base_lpN(:, 2), base_lpN(:, 6)];
draw_frame(R_lp, base_T_lft(1:3, 4));
% Draw frame for the right gripper plate
R_rp = [base_rpN(:, 4), base_rpN(:, 2), base_rpN(:, 6)];
draw_frame(R_rp, base_T_rft(1:3, 4));

xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
% xlim([0.80, 0.90]);
% ylim([0.35, 0.50]);
% zlim([0.0, 0.1]);
title("Pre-grasp positioning task with cylindrical peg");
view(98, 34);

%%
%//////////////////////////////////////////////////////////////////////%
%/////////////////////// HELPER FUNCTIONS /////////////////////////////%
%//////////////////////////////////////////////////////////////////////%
function [base_lp_vertx, base_rp_vertx, min_dl, min_dr] = check_success(qang, fixed_T, data)
    global cyn_pt;
    global base_T_lft, global base_T_rft;
    global base_lpN, global base_rpN;
    
    % Transformations of left and right finger tips wrt gripper_base frame
    gb_T_lft = fixed_T.gb_T_lf * fixed_T.lf_T_lft;
    gb_T_rft = fixed_T.gb_T_rf * fixed_T.rf_T_rft;

    % Solve forward kinematics
    all_transformations = mycls.forward_kinematics(qang, 'left_gripper_base');
    base_T_gb = all_transformations(:, :, end);

    % Compute base to left and right finger tip transformations
    base_T_lft = base_T_gb * gb_T_lft;
    base_T_rft = base_T_gb * gb_T_rft;

    % Transform all the normals wrt base frame
    base_lpN = base_T_lft(1:3, 1:3) * data.lp_nrm';
    base_rpN = base_T_rft(1:3, 1:3) * data.rp_nrm';

    % Transform all the vertices wrt base frame
    % left plate
    base_lp_vertx = base_T_lft * [data.lp_vrtx'; ones(1, size(data.lp_vrtx, 1))];
    base_lp_vertx = base_lp_vertx(1:3, :);
    lp_vertx_initial = base_lp_vertx(:, 1);
    
    % right plate
    base_rp_vertx = base_T_rft * [data.rp_vrtx'; ones(1, size(data.rp_vrtx, 1))];
    base_rp_vertx = base_rp_vertx(1:3, :);
    rp_vertx_initial = base_rp_vertx(:, 1);
    
    % Compute b vector for left plate, right plate, block
    base_lp_bvec = cmp_bvec(base_lpN, base_lp_vertx);
    base_rp_bvec = cmp_bvec(base_rpN, base_rp_vertx);

    % Check collision between left plate and block
    x0_l = [lp_vertx_initial; cyn_pt];
    [x_lp, x_peg, min_dl] = check_colli(base_lpN, base_lp_bvec, x0_l);
    fprintf("The closest pair of points: \n");
    fprintf("x_lp: "); disp(x_lp');
    fprintf("x_peg: "); disp(x_peg');
    fprintf("Minimum distance: "); disp(min_dl);
    
    % Check collision between right plate and block
    x0_r = [rp_vertx_initial; cyn_pt];
    [x_rp, x_peg, min_dr] = check_colli(base_rpN, base_rp_bvec, x0_r);
    fprintf("The closest pair of points: \n");
    fprintf("x_rp: "); disp(x_rp');
    fprintf("x_peg: "); disp(x_peg');
    fprintf("Minimum distance: "); disp(min_dr);
end

%% computes b values for all the planes
function [bvec] = cmp_bvec(w, vrtx)
    bvec = [dot(w(:, 1), vrtx(:, 2));
        dot(w(:, 2), vrtx(:, 2));
        dot(w(:, 3), vrtx(:, 2));
        dot(w(:, 4), vrtx(:, 7));
        dot(w(:, 5), vrtx(:, 7));
        dot(w(:, 6), vrtx(:, 7))];
end

%% check collision between blocks
function [x1, x2, min_d] = check_colli(blk1_N, blk1_b, x0)
    global spawn_pos, global hc;
    
    % Objective function to minimize
    obj = @(x)norm(x(1:3) - x(4:6))^2;
    
    % Build-up the constraints
    A = [blk1_N', zeros(6, 3); zeros(2, 5), [1; -1]];
    b = [blk1_b; spawn_pos(3)+hc; -spawn_pos(3)];
    nonlcon = @disk_constraint;
    opts = optimoptions('fmincon','Algorithm','interior-point', 'Display', 'off');
    
    % Call MATLAB's built-in fmincon function
    x = fmincon(obj, x0, A, b, [], [], [], [], nonlcon, opts);
    
    % Separate the points on the gripper plate and the peg
    x1 = x(1:3);
    x2 = x(4:6);
    
    % Find the closest distance between the two objects
    min_d = norm(x1 - x2);
end

%% circular cross section constraint of the peg to be used with fmincon
function [c,ceq] = disk_constraint(x)
    global spawn_pos, global rc;
    c = (x(4) - spawn_pos(1))^2 + (x(5) - spawn_pos(2))^2 - (rc)^2;
    ceq = [];
end

%% Function that plots the gripper plates
function poly_rectangle(p1, p2, p3, p4, ele)
    % The points must be in the correct sequence.
    % The coordinates must consider x, y and z-axes.
    x = [p1(1) p2(1) p3(1) p4(1)];
    y = [p1(2) p2(2) p3(2) p4(2)];
    z = [p1(3) p2(3) p3(3) p4(3)];
    if ele == 1
        colr = 'r';
    else
%         colr = 'b';
        colr = [0.5, 0.5, 0.5];
    end
    fill3(x, y, z, colr, 'Facealpha', 0.7);
    hold on
end

%% Draw 3D Box
function box3d(pt, ele)
    poly_rectangle(pt(:, 1), pt(:, 2), pt(:, 3), pt(:, 4), ele);
    poly_rectangle(pt(:, 1), pt(:, 2), pt(:, 5), pt(:, 8), ele);
    poly_rectangle(pt(:, 5), pt(:, 6), pt(:, 7), pt(:, 8), ele);
    poly_rectangle(pt(:, 3), pt(:, 4), pt(:, 7), pt(:, 6), ele);
    poly_rectangle(pt(:, 2), pt(:, 3), pt(:, 6), pt(:, 5), ele);
    poly_rectangle(pt(:, 1), pt(:, 4), pt(:, 7), pt(:, 8), ele);
end

%% Draw frame
function draw_frame(R, p)
    quiver3(p(1), p(2), p(3), R(1, 1), R(2, 1), R(3, 1), 0.025, 'r', 'LineWidth', 2.0);
    quiver3(p(1), p(2), p(3), R(1, 2), R(2, 2), R(3, 2), 0.025, 'g', 'LineWidth', 2.0);
    quiver3(p(1), p(2), p(3), R(1, 3), R(2, 3), R(3, 3), 0.025, 'b', 'LineWidth', 2.0);
end
