clc
clear
close all

addpath ('../utilities');

% Number of trials
max_run = 1000;
max_epoch = 1;

% Decide to experiemnt with Robust or Worst Solution
% correct relative orientation of block and plates
ang = 0.07;

% Transformation of the block with respect to base frame
ornt_blk = rot_z(ang);
% spawn_pos = [0.70479; 0.39129; -0.13244+(-0.04)];
spawn_pos = [0.71305; 0.378638; 0.30+(-0.05)];
base_T_block = [ornt_blk(1:3, 1:3) spawn_pos; zeros(1, 3) 1];

% Get the approach and grasp configuration joint space
% robust_sol = [0.0329 -0.2080 -1.0001 1.4125  1.2341 1.0239 2.6237];
% worst_sol = [-0.2826 -0.5119 -0.4362 1.4440  0.5445 0.7307 2.9322];
robust_sol = [0.0052   -0.1660   -2.0927    1.1777    1.6105    2.0793    2.6467];
worst_sol = [-0.1565   -1.0611   -0.7174    1.1818    0.3113    1.5296    3.0430];

% Decide Gaussian Noise Parameters
mu_val = 0; std_val = 0.0045;
% lb = -0.011;
% ub = 0.011;
lb = -100;
ub = 100;

%///////////////////////////////////////////////////////////////////////%
%///////////////////////////////////////////////////////////////////////%
%///////////////////// DO NOT TOUCH ////////////////////////////////////%
%///////////////////////////////////////////////////////////////////////%
%///////////////////////////////////////////////////////////////////////%
rs = zeros(1, max_epoch);
ws = zeros(1, max_epoch);

% Load data
data = load('coord_normals.mat');
fixed_T = load('fixed_trns.mat');
for epoch = 1:max_epoch
    r_success = 0;
    w_success = 0;
    for num_run = 1:max_run
        % Generate gaussian noise. Input order: (mean, std, lb, ub, sz)
        gn = mycls.gauss_noise(mu_val, std_val, lb, ub, 7);

        % Add noise with desired configuration
        qang_r = robust_sol + gn;
        qang_w = worst_sol + gn;

        % Check collision with robust and worst solutions
        [success_cnt, base_blk_vertx, base_lp_vertx, base_rp_vertx] = check_success(qang_r, fixed_T, data, base_T_block);
        r_success = r_success + success_cnt;
        [success_cnt, base_blk_vertx, base_lp_vertx, base_rp_vertx] = check_success(qang_w, fixed_T, data, base_T_block);
        w_success = w_success + success_cnt;
    end
    rs(1, epoch) = r_success;
    ws(1, epoch) = w_success;
end

fprintf('Success (robust): %d\n', r_success);
fprintf('Success (worst): %d\n', w_success);

figure(1)
pts = 1:max_epoch;
plot(pts, rs, 'o-', pts, ws, 'o-');
% hold on
% plot(pts, rs, 'bo', pts, ws, 'ro');
xlabel('Epochs')
ylabel('Success count')
title('Robust and Worst solution performance')
grid on
legend('robust solution', 'worst solution', 'location', 'south')

% Plot box and gripper location in 2D
bx_data = base_blk_vertx(1:2, [7 4 3 6 7]);
lp_data = base_lp_vertx(1:2, [7 4 3 6 7]);
rp_data = base_rp_vertx(1:2, [7 4 3 6 7]);

figure(2)
patch(bx_data(1, :), bx_data(2, :), 'r');
hold on
patch(lp_data(1, :), lp_data(2, :), 'b');
hold on
patch(rp_data(1, :), rp_data(2, :), 'b')
grid on
xlabel('x base')
ylabel('y base')
axis ('square')
legend('box', 'lp', 'rp')
title('Collision check')

% % Plot 3D shapes
figure(3)
box3d(base_blk_vertx, 1);
box3d(base_lp_vertx, 2);
box3d(base_rp_vertx, 2);
view(-127, 2);
%% 


%//////////////////////////////////////////////////////////////////////%
%/////////////////////// HELPER FUNCTIONS /////////////////////////////%
%//////////////////////////////////////////////////////////////////////%
% computes b values for all the planes
function [bvec] = cmp_bvec(w, vrtx)
    bvec = [dot(w(:, 1), vrtx(:, 2));
        dot(w(:, 2), vrtx(:, 2));
        dot(w(:, 3), vrtx(:, 2));
        dot(w(:, 4), vrtx(:, 7));
        dot(w(:, 5), vrtx(:, 7));
        dot(w(:, 6), vrtx(:, 7))];
end

% check collision between blocks
function [x1, x2, min_d] = check_colli(blk1_N, blk1_b, blk2_N, blk2_b)
    A = [blk1_N' zeros(6, 3); zeros(6, 3) blk2_N'];
    b = [blk1_b; blk2_b];
    f = zeros(6, 1);
    H = diag(ones(1, 6));
    H(1:3, 4:end) = diag(-1*ones(1, 3));
    H(4:end, 1:3) = diag(-1*ones(1, 3));

    opts = optimset('Display', 'off');
    x = quadprog(H, f, A, b, [], [], [], [], [], opts);             % MATLAB's quadprog() function
    x1 = x(1:3); x2 = x(4:6);
    fvalue = 0.5*(x'*H*x) + f'*x;         % objective value
    fprintf('Objective value: %6.8f\n', fvalue);
    min_d = norm(x(1:3)-x(4:6));
    fprintf('Minimum dist between two blocks: %6.8f\n', min_d);
end

% Draw 3D Box
function box3d(pt, ele)
    poly_rectangle(pt(:, 1), pt(:, 2), pt(:, 3), pt(:, 4), ele);
    poly_rectangle(pt(:, 1), pt(:, 2), pt(:, 5), pt(:, 8), ele);
    poly_rectangle(pt(:, 5), pt(:, 6), pt(:, 7), pt(:, 8), ele);
    poly_rectangle(pt(:, 3), pt(:, 4), pt(:, 7), pt(:, 6), ele);
    poly_rectangle(pt(:, 2), pt(:, 3), pt(:, 6), pt(:, 5), ele);
    poly_rectangle(pt(:, 1), pt(:, 4), pt(:, 7), pt(:, 8), ele);
end

function poly_rectangle(p1, p2, p3, p4, ele)
    % The points must be in the correct sequence.
    % The coordinates must consider x, y and z-axes.
    x = [p1(1) p2(1) p3(1) p4(1)];
    y = [p1(2) p2(2) p3(2) p4(2)];
    z = [p1(3) p2(3) p3(3) p4(3)];
    if ele == 1
        colr = 'r';
    else
        colr = 'b';
    end
    fill3(x, y, z, colr);
    hold on
end

function [success_cnt, base_blk_vertx, base_lp_vertx, base_rp_vertx] = check_success(qang, fixed_T, data, base_T_block)
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
    base_bN = base_T_block(1:3, 1:3) * data.blk_nrm';
    base_lpN = base_T_lft(1:3, 1:3) * data.lp_nrm';
    base_rpN = base_T_rft(1:3, 1:3) * data.rp_nrm';

    % Transform all the vertices wrt base frame
    % left plate
    base_lp_vertx = base_T_lft * [data.lp_vrtx'; ones(1, size(data.lp_vrtx, 1))];
    base_lp_vertx = base_lp_vertx(1:3, :);
    % right plate
    base_rp_vertx = base_T_rft * [data.rp_vrtx'; ones(1, size(data.rp_vrtx, 1))];
    base_rp_vertx = base_rp_vertx(1:3, :);
    % block
    base_blk_vertx = base_T_block * [data.blk_vrtx'; ones(1, size(data.blk_vrtx, 1))];
    base_blk_vertx = base_blk_vertx(1:3, :);

    % Compute b vector for left plate, right plate, block
    base_blk_bvec = cmp_bvec(base_bN, base_blk_vertx);
    base_lp_bvec = cmp_bvec(base_lpN, base_lp_vertx);
    base_rp_bvec = cmp_bvec(base_rpN, base_rp_vertx);

    % Check collision between left plate and block
    [x_lp, x_lblk, min_dl] = check_colli(base_lpN, base_lp_bvec, base_bN, base_blk_bvec);
    % Check collision between right plate and block
    [x_rp, x_rblk, min_dr] = check_colli(base_rpN, base_rp_bvec, base_bN, base_blk_bvec);

    % Count success
    if min_dl > 0.2*1e-4 && min_dr > 0.2*1e-4
        success_cnt = 1;
    else
        success_cnt = 0;
    end
end
