% This code computes all the Jacobians
function [jacmat] = cmpAllJacs(sols, robot, robot_base)
    ak = robot(1, :);
    alpk = robot(2, :);
    dk = robot(3, :);
    
    % Compute all the Jacobians
    for ii = 1:size(sols, 1)
        th = sols(ii, :);
        % Solve forward kinematics
        T = zeros(4, 4, length(ak)+1);
        T(:, :, 1) = robot_base;
        for i = 1:length(ak)
            T(:, :, i+1) = T(:, :, i)*lcl_trns(ak(i), alpk(i), dk(i), th(i));
        end
        % AnaJac(:, :, ii) = cmpJac(T);
        jacmat(:, :, ii) = mycls.jacb_spatial(T(:, :, 2:end), T(:, :, 1));
    end
end

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
