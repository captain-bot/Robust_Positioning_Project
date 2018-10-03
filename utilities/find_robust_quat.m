function [wrstquat, wrstang] = find_robust_quat(quat_des, solIKs, mu, sigma, num_std, rbt, rbt_base)
    rs = 0.001;
    fprintf('Cone-top discretization resolution: %2.4f radians\n', rs);
    
    % Compute c (squared radius of joint space error ball)
    c = (num_std*sigma)^2;
    
    % Build the robot
    ak = rbt(1, :);
    alpk = rbt(2, :);
    dk = rbt(3, :);
    lnk = dk(1, 3);
    
    % Compute all the analytical Jacobians
    allJacs = cmpAllJacs(solIKs, rbt, rbt_base);
    
    % Compute robust quaternion
    H_qd = funcH(quat_des);
    a_vec = H_qd*quat_des';
    k_val = quat_des(4)/quat_des(3);

    start_time = cputime;
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

            [ellp_sat, obj, v_vec] = v_check(cone_rad, cone_axis, cone_ht, L_err, k_val, H_qd, rs);

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
            angle(i) = {'no'};
        else
            angle(i) = {diff_angle};
            worst_quat(i) = {quat_des' + H_qd'*opt_vvec};
            wrstquat(:, i) = quat_des' + H_qd'*opt_vvec;
            wrstang(1, i) = diff_angle(1, end);
        end
    end
    fprintf('Found robust quaternion: '); disp(wrstquat');
    fprintf('Included angle(deg): '); disp(acosd(quat_des*wrstquat));
end

%//////////////////////////////////////////////////////%
%     All supporting functions are below this line     %
%//////////////////////////////////////////////////////%

% This code computes all the Jacobians
% function [AnaJac] = cmpAllJacs(sols, robot)
%     ak = robot(1, :);
%     alpk = robot(2, :);
%     dk = robot(3, :);
%     
%     % Compute all the Jacobians
%     for ii = 1:size(sols, 1)
%         th = sols(ii, :);
%         % Solve forward kinematics
%         T = zeros(4, 4, 4);
%         T(:, :, 1) = eye(4, 4);
%         for i = 1:length(ak)
%             T(:, :, i+1) = T(:, :, i)*lcl_trns(ak(i), alpk(i), dk(i), th(i));
%         end
%         AnaJac(:, :, ii) = cmpJac(T);
%     end
% end

% Jacobian Function
% function [J] = cmpJac(T)
%     n = size(T, 3) - 1;
%     On = T(1:3, 4, end);
%     J = zeros(6, n);
%     
%     for i = 1:n
%         J(:, i) = [cross(T(1:3, 3, i), (On - T(1:3, 3, i)));
%                     T(1:3, 3, i)];
%     end
% end

% % Functions used to find robust solution
% function [ellp_stisfied, obj_val, vvec] = v_check(r, zp, scl, Lc, kvar, H, resl)
%     avec = -zp;
%     ellp_stisfied = 0;
%     obj_val = 0;
%     vvec = 0;
%     
%     % randomly initialize xp
%     % then normalize
%     zp = zp/norm(zp);
% 
%     x1 = 1; x2 = 1;
%     x3 = -(zp(1)*x1+zp(2)*x2)/zp(3);
%     xp = [x1; x2; x3]; xp = xp/norm(xp);
% 
% 
%     yp = cross(zp, xp);
%     rotmat = [xp yp zp];
%     
%     th = [0:resl:2*pi];
%     pts = zeros(3, length(th));
% 
%     pts(1, :) = r*cos(th);
%     pts(2, :) = r*sin(th);
% 
%     % transform the points into base frame from body frame
%     base_pts = rotmat*pts;          % rotate
%     base_pts = base_pts + scl * zp; % translate
% 
%     % check ellipsoid constraint
%     for i = 1:size(base_pts, 2)
%         if norm(Lc'*base_pts(:, i)) - 1 <= 1e-9
%             ellp_stisfied = 1;
%             objval = 1 + avec'*base_pts(:, i);
%             if objval > obj_val
%                 obj_val = objval;
%                 vvec = base_pts(:, i);
%             end
%         end
%     end
% end

% function [val] = funcH(q)
%     % as per our latest derivation
%     val = [q(1) q(4) -q(3) -q(2);
%            -q(4) q(1) q(2) -q(3);
%            q(3) -q(2) q(1) -q(4)];
% end
