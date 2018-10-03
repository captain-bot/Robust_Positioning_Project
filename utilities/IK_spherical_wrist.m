function [sol] = IK_spherical_wrist(x_dir)
    % Computes multiple IKs for spherical wrist such that x-axis of
    % rotation matrix is same as the desired direction.
    % Note: no joint limit is considered
    % Date modified: October 2nd, 2018

    % Step1: Define theta_3
    th3 = 0:0.1:2*pi;
    sol = [];
    for i = 1:length(th3)
        if abs(-x_dir(3)/cos(th3(i))) <= 1
            th1 = [];
            % Step2: Compute theta_2
            th2_1 = asin(-x_dir(3)/cos(th3(i)));
            th2_2 = pi - th2_1;

            % Step3: Compute theta_3
            nrm_val1 = sqrt((cos(th2_1)*cos(th3(i)))^2 + sin(th3(i))^2);
            nrm_val2 = sqrt((cos(th2_2)*cos(th3(i)))^2 + sin(th3(i))^2);

            alpha1 = atan2(sin(th3(i))/nrm_val1, cos(th2_1)*cos(th3(i))/nrm_val1);
            alpha2 = atan2(sin(th3(i))/nrm_val2, cos(th2_2)*cos(th3(i))/nrm_val2);

            th1(1) = acos(x_dir(1)/nrm_val1) - alpha1;
            th1(2) = acos(x_dir(1)/nrm_val2) - alpha2;
            th1(3) = 2*pi - acos(x_dir(1)/nrm_val1) - alpha1;
            th1(4) = 2*pi - acos(x_dir(1)/nrm_val2) - alpha2;
            
            for j = 1:4
                if j == 1 || j == 3
                    if sign(cos(th1(j))*sin(th3(i)) + cos(th2_1)*cos(th3(i))*sin(th1(j))) == sign(x_dir(2))   % check if second element of x_dir is matching
                        sol = [sol; [th1(j), th2_1, th3(i)]];
                    end
                 elseif j == 2 || j == 4
                     if sign(cos(th1(j))*sin(th3(i)) + cos(th2_2)*cos(th3(i))*sin(th1(j))) == sign(x_dir(2))  % check if second element of x_dir is matching
                        sol = [sol; [th1(j), th2_2, th3(i)]];
                     end
                 end
            end
        end
    end
    disp(sol);
end
