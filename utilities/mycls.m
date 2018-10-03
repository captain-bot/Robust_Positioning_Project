classdef mycls
    properties(Constant)
        base = translation(0.024645, 0.219645, 0.118588)*rot_z(pi/4)*translation(0.055695, 0, 0.011038);
        ak = [0.069 0 0.069 0 0.010 0 0];
        alpk = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0];
        dk = [0.27035 0 (0.102 + 0.26242) 0 (0.10359 + 0.2707) 0 (0.115975 + 0.11355)];
        
        % Provide the joint limits (clo1 -> min and col2 -> max)
        joint_limits = [-1.7016	1.7016;
                        -2.1470	1.047;
                        -3.0541	3.0541;
                        -0.0500	2.618;
                        -3.0590	3.059;
                        -1.5707	2.094;
                        -3.0590	3.059];
    end
    
    methods (Static)
        %///////////////////////////////////////////////////////////////
        %          Compute forward kinematics and jacobian
        %///////////////////////////////////////////////////////////////
        function [all_trns, jacobian_mat] = frdkin_jaco(ang, end_frame)
            % Compute forward kinematics
            transformation_mats = mycls.forward_kinematics(ang, end_frame);
            % Compute jacobian
            jacobian_mat = mycls.jacb_spatial(transformation_mats(:, :, 2:end), transformation_mats(:, :, 1));
        end
        
        %Solve forward kinematics
        function[all_trns] = forward_kinematics(ang, end_frame)
            if strcmp('left_gripper_base', end_frame)
                add_link = 0.025;
            elseif strcmp('left_gripper', end_frame)
                add_link = 0.15;
            elseif strcmp('IKFast_frame', end_frame)
                add_link = 0.05;
            end
            
            % Solve forward kinematics upto end_frame
            all_trns = zeros(4, 4, size(ang, 2) + 1);           
            trans_k = mycls.base;
            all_trns(:, : , 1) = trans_k;
            for i = 1 : length(mycls.ak)
                if i == 7
                    trans_k = trans_k * mycls.baxlcl_tran(mycls.ak(i), mycls.dk(i)+add_link, mycls.alpk(i), ang(i), i);
                else
                    trans_k = trans_k * mycls.baxlcl_tran(mycls.ak(i), mycls.dk(i), mycls.alpk(i), ang(i), i);
                end
                all_trns(:, : , i+1) = trans_k;
            end
        end
        
        %///////////////////////////////////////////////////////////
        %               Computes local relative transform
        %///////////////////////////////////////////////////////////
        function[trans_mat] = baxlcl_tran(a, d, alp, ang, i)
            if i == 2
                ang = ang + pi/2;
            end
            trans_mat = [cos(ang) -cos(alp)*sin(ang) sin(alp)*sin(ang)  a*cos(ang);
                        sin(ang) cos(alp)*cos(ang) -sin(alp)*cos(ang)  a*sin(ang);
                        0 sin(alp) cos(alp) d;
                        0 0 0 1];
        end
        
        %////////////////////////////////////////////////////////////////
        %                       Spatial jacobian
        %////////////////////////////////////////////////////////////////
        function [jacob] = jacb_spatial(H, base)
            % Compute Spatial Jacobian of a Manipulator
            % Reference : Vidyasagar, Spong
            % our jacoian is same as "jacob0" method of robotics toolbox
            n = size(H,3);
            z_i = zeros(3,n+1);
            o_i = zeros(3,n+1);
            jacob = zeros(6,n);

            z_i(:, 1) = base(1:3, 3);       % Z axis of first frame
            o_i(:, 1) = base(1:3, 4);       % origin of first frame

            for ii = 2:n+1
                z_i(:,ii) = H(1:3,3,ii-1);
                o_i(:,ii) = H(1:3,4,ii-1);
            end

            for ii = 2:n+1
                jacob(:,ii-1) = [cross(z_i(:,ii-1),o_i(:,n+1)-o_i(:,ii-1));z_i(:,ii-1)];
            end
        end
        
        %/////////////////////////////////////////////////////////////////
        %               Compute Robust Position Joint Angles
        %/////////////////////////////////////////////////////////////////
        function [robust_sol, ik_array, Iindex, min_max_val] = cmp_PRobust(end_frame, elbw_up)
            % Load IK Solutions
            % Note: we took solution that are 0.01 radians apart form their respetive            
            str_join = ["data_files/left_ik", num2str(solnum), ".txt"];
            str = join(str_join, "");
            ik_array = load(str);

            %-------------------------------------------------------------
            % Attention : Turn this part off if all solutions are required
            % If want to Work with Elbow Up solutions only
            if elbw_up
                ik_array = mycls.only_elbw_up(ik_array);
            end
            %-------------------------------------------------------------

            % Solve position_error_min problem
            max_eig = zeros(1, size(ik_array, 1));
            for ii = 1:size(ik_array, 1)
                [~, jacobian_mat] = mycls.frdkin_jaco(ik_array(ii, :), end_frame);
                [~, D] = eig(jacobian_mat(1:3, :)*jacobian_mat(1:3, :)');
                max_eig(1, ii) = max(diag(D));
            end
            [M, I_best] = min(max_eig);
            [M_max, I_worst] = max(max_eig);
            Iindex = [I_best I_worst];
            min_max_val = [M M_max];
            fprintf('Robust Solution: '); disp(ik_array(Iindex(1, 1), :));
            % Store robust solution
            robust_sol = ik_array(Iindex(1, 1), :);
        end
        
        %////////////////////////////////////////////////////////////////
        %       Robust Solution with respect to a given direction
        %////////////////////////////////////////////////////////////////
        function [robust_sol, ik_array, Iindex, min_max_val] = cmpDirRobust(end_frame, elbw_up, cval, v_dir)
            % Load IK Solutions
            % Note: we took solution that are 0.01 radians apart form their
            % respetive
            
            str_join = ["data_files/left_ik", num2str(1), ".txt"];
            str = join(str_join, "");
            ik_array = load(str);

            %-------------------------------------------------------------
            % Attention : Turn this part off if all solutions are required
            % If want to Work with Elbow Up solutions only
            if elbw_up
                ik_array = mycls.only_elbw_up(ik_array);
            end
            %-------------------------------------------------------------
                
            % Solve position_error_min problem
            max_len = zeros(1, size(ik_array, 1));
            for ii = 1:size(ik_array, 1)
                [~, jacobian_mat] = mycls.frdkin_jaco(ik_array(ii, :), end_frame);
                max_len(1, ii) = mycls.intrcept_line(jacobian_mat(1:3, :), cval, v_dir);
            end
            [M,I_best] = min(max_len);
            [M_max, I_worst] = max(max_len);
            Iindex = [I_best I_worst];
            min_max_val = [M M_max];
            fprintf('Robust Solution: '); disp(ik_array(Iindex(1, 1), :));
            fprintf('Worst Solution: '); disp(ik_array(Iindex(1, 2), :));
            % Store robust solution
            robust_sol = ik_array(Iindex(1, 1), :);
        end
        
        %////////////////////////////////////////////////////////////////
        %           Find max intercept along a direction
        %////////////////////////////////////////////////////////////////
        function[max_intrcpt] = intrcept_line(Jmat, cval, v)
            Lc = chol(inv(Jmat*Jmat')/cval, 'lower');
            w = Lc\v;
            max_intrcpt = norm(w);
        end
        
        %/////////////////////////////////////////////////////////////////
        %       Returns only elbow up solutions from all solution pool
        %/////////////////////////////////////////////////////////////////
        function [ik_sol] = only_elbw_up(ik_sol_array)
            ik_sol_array_idx = zeros(1, 1);
            k = 1;
            for i = 1 : size(ik_sol_array, 1)
                if ik_sol_array(i, 2) < -0.1 && ik_sol_array(i, 6) > 0 && ik_sol_array(i, 7) > 0  % && ik_sol_array(i, 7) < 0.2
                    ik_sol_array_idx(1, k) = i;
                    k = k + 1;
                end
            end
            ik_sol = ik_sol_array(ik_sol_array_idx, :);
        end
        
        %/////////////////////////////////////////////////////////////////
        % Given an end effector configuration from Gazebo wrt base
        % transform that specific to IK_Fast input
        % This function is to get end effector transformation for IK_Fast input
        %/////////////////////////////////////////////////////////////////
        function [ik_fast_trns1] = IK_Fast_input_homog(ee_trans, last_frame)
            % adjust base
            ik_fast_trns1 = (translation(0.024645, 0.219645, 0.108588) * rot_z(pi/4))\ee_trans;
            % adjust end effector
            % if ee_trans is upto left_gripper frame ex_len = 0.15 - 0.05
            % if ee_trans is upto gripper_base frame ex_len = 0.025 - 0.05            
            if strcmp('left_gripper', last_frame)
                ex_len = 0.15 - 0.05;
            elseif strcmp('left_gripper_base', last_frame)
                ex_len = 0.025 - 0.05;
            end
            ik_fast_trns1(1:3, 4) = ik_fast_trns1(1:3, 4) - ik_fast_trns1(1:3, 1:3)*[0; 0; ex_len];
        end
        
        %////////////////////////////////////////////////////////////////////////
        % This function is same as the above except it takes (7 by 1) vector as
        % end effector transformation as in Gazebo
        % input format : ee_trans2 = [qw qx qy qz X Y Z];
        %////////////////////////////////////////////////////////////////////////
        function [ik_fast_trns2] = IK_Fast_input_quat(ee_trans2, last_frame)
            rotmat = quat2rotm(ee_trans2(1:4));
            ik_fast_trns2 = (translation(0.024645, 0.219645, 0.118588) * rot_z(pi/4)) \ [rotmat ee_trans2(5:7)'; zeros(1, 3) 1];
            % adjust end effector
            % if ee_trans is upto left_gripper frame ex_len = 0.15 - 0.05
            % if ee_trans is upto gripper_base frame ex_len = 0.025 - 0.05
            if strcmp('left_gripper', last_frame)
                ex_len = 0.15 - 0.05;
            elseif strcmp('left_gripper_base', last_frame)
                ex_len = 0.025 - 0.05;
            end
            % adjust end effector
            ik_fast_trns2(1:3, 4) = ik_fast_trns2(1:3, 4) - ik_fast_trns2(1:3, 1:3)*[0; 0; ex_len];
        end
        
        %////////////////////////////////////////////////////////////
        %               Adjust input to myIK algorithm
        %////////////////////////////////////////////////////////////
        function [myik_trns] = MyIK_input(ee_trans, last_frame)
            rotmat = quat2rotm(ee_trans(1:4));
            myik_trns = [rotmat ee_trans(5:7)'; zeros(1, 3) 1];
            if strcmp('left_gripper', last_frame)
                ex_len = 0.15 - 0.025;
            elseif strcmp('left_gripper_base', last_frame)
                ex_len = 0;
            end
            % adjust end effector
            myik_trns(1:3, 4) = myik_trns(1:3, 4) - myik_trns(1:3, 1:3)*[0; 0; ex_len];
        end
        
        %////////////////////////////////////////////////////////////////
        %               Generate Gaussian random noise
        % mn = mean; sig = sigma; lb = lower bound, ub = upper bound; sz =
        % size of random array
        %////////////////////////////////////////////////////////////////
        function [r] = gauss_noise(mn, sig, lb, up, sz)
            r = (lb-0.01)*ones(1, sz);
            for i = 1:length(r)
                while (r(1, i) > up || r(1, i) < lb)
                    r(1, i) = normrnd(mn, sig);
                end
            end
        end
        
        % ///////////////////////////////////////////////
        %               Check Joint Limits
        %////////////////////////////////////////////////
        function [] = check_jl(array_sol)
            limit_flag = 0;
            valid_sol = 0;
            valid_sol_array = zeros(1, size(array_sol, 2));
            for i = 1:size(array_sol, 1)
                for j = 1:size(array_sol, 2)
                    % check the limits
                    if array_sol(i, j)-tol < mycls.limits(j, 1) || array_sol(i, j)+tol > mycls.limits(j, 2)
                        fprintf('joint %d limit exceeded ', j);
                        fprintf('solution number %d\n', i);
                        limit_flag = 1;
                        % break;
                    end
                end
                if limit_flag == 0
                    fprintf('Solution %d limit ok !\n', i);
                    valid_sol = valid_sol + 1;
                    valid_sol_array(valid_sol, :) = array_sol(i, :);
                end
                % reset limit flag
                limit_flag = 0;
            end
            fileID = fopen('sol_valid2.txt','w');
            fprintf(fileID,'%6.8f %6.8f %6.8f %6.8f %6.8f %6.8f %6.8f\r\n', valid_sol_array');
            fclose(fileID);
        end
        
    end
end