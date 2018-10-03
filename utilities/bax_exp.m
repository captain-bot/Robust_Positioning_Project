classdef bax_exp
    properties (Constant)
        % transl(0.024645, 0.219645, 0.118588)
        % base = transl(0.025447, 0.21903, 0.10798) * trotz(pi/4) * transl(0.055695, 0, 0.011038);
        base = translation(0.024645, 0.219645, 0.118588)*rot_z(pi/4)*translation(0.055695, 0, 0.011038);
        ak = [0.069 0 0.069 0 0.010 0 0];
        alpk = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0];
        dk = [0.27035 0 (0.102 + 0.26242) 0 (0.10359 + 0.2707) 0 (0.115975 + 0.11355)];
        
        % dk = [0.27035 0 (0.102 + 0.26242) 0 (0.10359 + 0.2707) 0 (0.115975 + 0.11355) + 0.15];      % add 0.15 to match left_gripper transform
        % dk = [0.27035 0 (0.102 + 0.26242) 0 (0.10359 + 0.2707) 0 (0.115975 + 0.11355) + 0.025];       % add 0.025 to match left_gripper_base    
        % dk = [0.27035 0 (0.102 + 0.26242) 0 (0.10359 + 0.2707) 0 (0.115975 + 0.11355) + 0.05];        % 0.05 is added to mathch with demoik.cpp transformation
        
        % Provide the joint limits (clo1 -> min and col2 -> max)
        limits = [-1.7016	1.7016;
                  -2.147	1.047;
                  -3.0541	3.0541;
                  -0.05	    2.618;
                  -3.059	3.059;
                  -1.5707	2.094;
                  -3.059	3.059];
    end
    
    methods (Static)
        % Compute forward kinematics
        function [trans_k, all_trns] = frd_kin(ang)
            all_trns = zeros(4, 4, size(ang, 2) + 1);           
            trans_k = bax_exp.base;
            all_trns(:, : , 1) = trans_k;
            for i = 1 : length(bax_exp.ak)
                trans_k = trans_k * bax_exp.baxlcl_tran(bax_exp.ak(i), bax_exp.dk(i), bax_exp.alpk(i), ang(i), i);
                all_trns(:, : , i+1) = trans_k;
            end
        end
        
        % Support Function : Use to compute local relative transform
        function[trans_mat] = baxlcl_tran(a, d, alp, ang, i)
            if i == 2
                ang = ang + pi/2;
            end
            trans_mat = [cos(ang) -cos(alp)*sin(ang) sin(alp)*sin(ang)  a*cos(ang);
                        sin(ang) cos(alp)*cos(ang) -sin(alp)*cos(ang)  a*sin(ang);
                        0 sin(alp) cos(alp) d;
                        0 0 0 1];
        end
        
        % Construct robot model for Robotics Toolbox
        function [Baxter_l] = const_robot(end_frame)
            if strcmp('left_gripper_base', end_frame)
                add_link = 0.025;
            elseif strcmp('left_gripper', end_frame)
                add_link = 0.15;
            elseif strcmp('IKFast_frame', end_frame)
                add_link = 0.05;
            end
            % Defining the links for toolbox (Left Arm)        
            Ll(1) = Link ([0    bax_exp.dk(1)  bax_exp.ak(1)      bax_exp.alpk(1)  0    0]);     % start at joint s0 and move to joint s1
            Ll(2) = Link ([0    0              0                  bax_exp.alpk(2)  0    pi/2]);  % start at joint s1 and move to joint e0
            Ll(3) = Link ([0    bax_exp.dk(3)  bax_exp.ak(3)      bax_exp.alpk(3)  0    0]);     % start at joint e0 and move to joint e1
            Ll(4) = Link ([0    0              0                  bax_exp.alpk(4)  0    0]);     % start at joint e1 and move to joint w0
            Ll(5) = Link ([0    bax_exp.dk(5)  bax_exp.ak(5)      bax_exp.alpk(5)  0    0]);     % start at joint w0 and move to joint w1
            Ll(6) = Link ([0    0              0                  bax_exp.alpk(6)  0    0]);     % start at joint w1 and move to joint w2
            Ll(7) = Link ([0    bax_exp.dk(7)+add_link  0                  bax_exp.alpk(7)  0    0]);     % start at joint w2 and move to end-effector
            
            % Left arm of Baxter Robot
            Baxter_l = SerialLink(Ll, 'name', 'Baxter_L', 'base' , transl(0.024645, 0.219645, 0.118588) * trotz(pi/4) * transl(0.055695, 0, 0.011038));
            % Baxter_l = SerialLink(Ll, 'name', 'Baxter_L', 'base' , transl(0) * trotz(0) * transl(0.055695, 0, 0.011038));
        end
        
        % Compute Forward Kinematics and Jacobian
        function[fk, jacb] = kin_and_jac(ang, end_frame)
            Bax_l = bax_exp.const_robot(end_frame);
            fk = Bax_l.fkine(ang);
            jacb = Bax_l.jacob0(ang);
        end
        
        % Compute Forward Kinematics
        function[fk] = kin(ang, end_frame)
            Bax_l = bax_exp.const_robot(end_frame);
            frd = Bax_l.fkine(ang);
            fk = [frd.n frd.o frd.a frd.t];
        end
        
        % Compute Jacobian
        function[jacb] = jac(ang, end_frame)
            Bax_l = bax_exp.const_robot(end_frame);
            jacb = Bax_l.jacob0(ang);
        end
        
        % Compute Robust Position Joint Angles
        function [robust_sol, ik_array, Iindex, min_max_val, Bax_l] = cmp_PRobust(end_frame, elbw_up)
            maxsol = 1;
            % First construct the robot
            Bax_l = bax_exp.const_robot(end_frame);
            % Load IK Solutions
            % Note: we took solution that are 0.01 radians apart form their respetive            
            robust_sol = zeros(maxsol, 7);
            for solnum = 1:maxsol
                str_join = ["data_files/left_ik", num2str(solnum), ".txt"];
                str = join(str_join, "");
                ik_array = load(str);

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Attention : Turn this part off if all solutions are required
                % If want to Work with Elbow Up solutions only
                if elbw_up
                    ik_array = bax_exp.only_elbw_up(ik_array);
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % Solve position_error_min problem
                max_eig = zeros(1, size(ik_array, 1));
                bax_jacb = zeros(6, size(bax_exp.ak, 2), size(ik_array, 1));
                for ii = 1:size(ik_array, 1)
                    bax_jacb(:, :, ii) = Bax_l.jacob0(ik_array(ii, :));
                    [V, D] = eig(bax_jacb(1:3, :, ii)*bax_jacb(1:3, :, ii)');
                    max_eig(1, ii) = max(diag(D));
                end
                [M,I_best] = min(max_eig);
                [M_max, I_worst] = max(max_eig);
                Iindex = [I_best I_worst];
                min_max_val = [M M_max];
                fprintf('Robust Solution: '); disp(ik_array(Iindex(1, 1), :));

                % Store robust solution
                robust_sol(solnum, :) = ik_array(Iindex(1, 1), :);
            end

            % Write all the robust solutions in a text file
            fileID = fopen(join([str_join(1), "robustsol.txt"], ""), 'w');
            fprintf(fileID,'%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f\r\n', robust_sol');
            fclose(fileID);
        end
        
        % Robust Solution with respect to a given direction
        function [robust_sol, ik_array, Iindex, min_max_val, Bax_l] = cmpDirRobust(end_frame, elbw_up, cval, v_dir)
            maxsol = 1;
            % First construct the robot
            Bax_l = bax_exp.const_robot(end_frame);
            % Load IK Solutions
            % Note: we took solution that are 0.01 radians apart form their respetive            
            robust_sol = zeros(maxsol, 7);
            for solnum = 1:maxsol
                str_join = ["data_files/left_ik", num2str(solnum), ".txt"];
                str = join(str_join, "");
                ik_array = load(str);

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Attention : Turn this part off if all solutions are required
                % If want to Work with Elbow Up solutions only
                if elbw_up
                    ik_array = bax_exp.only_elbw_up(ik_array);
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % Solve position_error_min problem
                max_len = zeros(1, size(ik_array, 1));
                for ii = 1:size(ik_array, 1)
                    PosJac = Bax_l.jacob0(ik_array(ii, :));
                    max_len(1, ii) = bax_exp.intrcept_line(PosJac(1:3, :), cval, v_dir);
                end
                [M,I_best] = min(max_len);
                [M_max, I_worst] = max(max_len);
                Iindex = [I_best I_worst];
                min_max_val = [M M_max];
                fprintf('Robust Solution: '); disp(ik_array(Iindex(1, 1), :));
                fprintf('Worst Solution: '); disp(ik_array(Iindex(1, 2), :));

                % Store robust solution
                robust_sol(solnum, :) = ik_array(Iindex(1, 1), :);
            end
        end
        
        % Compute Dimensions of Error Ellipsoid
        function [major_len, minor_len] = cmp_ErrorEllp(cval, end_frame, elbw_up)
            [~, ik_array, Iindex, ~, Baxter_l] = bax_exp.cmp_PRobust(end_frame, elbw_up);
            % Compute Error Ellipsoid at End-Effector (both BEST and WORST case)
            for i = 1:length(Iindex)
                Jposition = Baxter_l.jacob0(ik_array(Iindex(1, i), :));
                Amat = inv(Jposition(1:3, :)*Jposition(1:3, :)');
                [major_len, minor_len] = bax_exp.elplot(Amat, sqrt(cval));
                if i == 1
                    fprintf('Robust Solution: '); disp(ik_array(Iindex(1, i), :));
                else
                    fprintf('Worst Solution: '); disp(ik_array(Iindex(1, i), :));
                end
                fprintf('Major Axis Length: %2.6f\n', major_len);
                fprintf('Minor Axis Length: %2.6f\n', minor_len);
                fprintf('\n\n');
            end
        end
        
        % Compute major and minor axis of error ellipsoid
        function [a, b] = elplot(A, rBall)  
              %plot error ellipse
              [~, D] = eig(A);
              [b, ~] = max(abs(diag(D)));
              [a, ~] = min(abs(diag(D)));
              a = rBall/sqrt(a);                % scaling major axis
              b = rBall/sqrt(b);                % scaling minor axis
        end
        
        % Returns only elbow up solutions from all solution pool
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
        
        % Given an end effector configuration from Gazebo wrt base
        % transform that specific to IK_Fast input
        % This function is to get end effector transformation for IK_Fast input
        function [ik_fast_trns1] = IK_Fast_input_homog(ee_trans, last_frame)
            % adjust base
            % ik_fast_trns1 = inv(transl(0.024645, 0.219645, 0.108588) * trotz(pi/4))*ee_trans;
            ik_fast_trns1 = inv(translation(0.024645, 0.219645, 0.108588) * rot_z(pi/4))*ee_trans;
	    % adjust end effector
            % if ee_trans is upto left_gripper frame ex_len = 0.15 - 0.05
            % if ee_trans is upto gripper_base frame ex_len = 0.025 - 0.05
            if last_frame == 'left_gripper'
                ex_len = 0.15 - 0.05;
            elseif last_frame == 'left_gripper_base'
                ex_len = 0.025 - 0.05;
            end
            ik_fast_trns1(1:3, 4) = ik_fast_trns1(1:3, 4) - ik_fast_trns1(1:3, 1:3)*[0; 0; ex_len];
        end

        % This function is same as the above except it takes (7 by 1) vector as
        % end effector transformation as in Gazebo
        % input method : ee_trans2 = [qw qx qy qz X Y Z];
        function [ik_fast_trns2] = IK_Fast_input_quat(ee_trans2, last_frame)
            rotmat = quat2rotm(ee_trans2(1:4));
            % ik_fast_trns2 = inv(transl(0.024645, 0.219645, 0.118588) * trotz(pi/4))*[rotmat ee_trans2(5:7)'; zeros(1, 3) 1];
            ik_fast_trns2 = inv(translation(0.024645, 0.219645, 0.118588) * rot_z(pi/4)) * [rotmat ee_trans2(5:7)'; zeros(1, 3) 1];
            % adjust end effector
            % if ee_trans is upto left_gripper frame ex_len = 0.15 - 0.05
            % if ee_trans is upto gripper_base frame ex_len = 0.025 - 0.05
            if last_frame == 'left_gripper'
                ex_len = 0.15 - 0.05;
            elseif last_frame == 'left_gripper_base'
                ex_len = 0.025 - 0.05;
            end
            % adjust end effector
            ik_fast_trns2(1:3, 4) = ik_fast_trns2(1:3, 4) - ik_fast_trns2(1:3, 1:3)*[0; 0; ex_len];
        end
        
        
        function [myik_trns] = MyIK_input(ee_trans, last_frame)
            rotmat = quat2rotm(ee_trans(1:4));
            myik_trns = [rotmat ee_trans(5:7)'; zeros(1, 3) 1];
            if last_frame == 'left_gripper'
                ex_len = 0.15 - 0.025;
            elseif last_frame == 'left_gripper_base'
                ex_len = 0;
            end
            % adjust end effector
            myik_trns(1:3, 4) = myik_trns(1:3, 4) - myik_trns(1:3, 1:3)*[0; 0; ex_len];
        end
        
        % Generate Gaussian random noise
        % mn = mean; sig = sigma; lb = lower bound, ub = upper bound; sz =
        % size of random array
        function [r] = gauss_noise(mn, sig, lb, up, sz)
            r = (lb-0.01)*ones(1, sz);
            for i = 1:length(r)
                while (r(1, i) > up || r(1, i) < lb)
                    r(1, i) = normrnd(mn, sig);
                end
            end
        end
        
        % Check Joint Limits
        function [] = check_jl(array_sol)
            limit_flag = 0;
            valid_sol = 0;
            valid_sol_array = zeros(1, size(array_sol, 2));
            for i = 1:size(array_sol, 1)
                for j = 1:size(array_sol, 2)
                    % check the limits
                    if array_sol(i, j)-tol < bax_exp.limits(j, 1) || array_sol(i, j)+tol > bax_exp.limits(j, 2)
                        fprintf('joint %d limit exceeded ', j);
                        fprintf('solution number %d\n', i);
                        limit_flag = 1;
                        % break;
                    end
                end
                if limit_flag == 0
                    fprintf('Solution %d ok !\n', i);
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
        
        % Find max intercept along a direction
        function[max_intrcpt] = intrcept_line(Jmat, cval, v)
            L = chol(inv(Jmat*Jmat')/cval, 'lower');
            w = inv(L)*v;
            max_intrcpt = norm(w);
        end
    end
end
