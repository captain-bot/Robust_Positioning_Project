function [ theta_1,  theta_2, val] = RR_inverse(Link_Lengths, pose)
% Inverse Position Kinematics

L1 = Link_Lengths(1);
L2 = Link_Lengths(2);
x = pose(1);
y = pose(2);

a = (x^2 + y^2 -L1^2 - L2^2)/(2*L1*L2);

if -1 <= a && a <= 1
%  if (L1 + L2)^2 - (x^2 + y^2) > 0
    c2 = (x^2 + y^2 - (L1^2 + L2^2))/(2*L1*L2);
    s21 = sqrt(1 - c2^2);
    s22 = -sqrt(1 - c2^2);
    theta21 = atan2(s21,c2);
    theta22 = atan2(s22,c2);
    theta11 = atan2(y,x) - atan2(L2*sin(theta21), L1 + L2*cos(theta21));
    theta12 = atan2(y,x) - atan2(L2*sin(theta22), L1 + L2*cos(theta22));
    
    % Ensuring that theta1 lies between -pi and pi
    if (theta11 >= pi)
        theta11 = pi - theta11;
    end
    if (theta12 >= pi)
        theta12 = pi - theta12;
    end
    
    theta_2 = [theta12, theta22];

    theta_1 = [theta11, theta21];
    val = 'Y';
   
else
    theta_1 = ['e' 'e']; theta_2 = ['e' 'e'];
    val = 'N';
%     error('Point is not reachable by manipulator');
end
end