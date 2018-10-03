function [T] = lcl_trns(a, alp, d, ang)
    % Local transformation function
    T = [cos(ang) -sin(ang)*cos(alp) sin(ang)*sin(alp)  a*cos(ang);
         sin(ang)  cos(ang)*cos(alp) -cos(ang)*sin(alp) a*sin(ang);
                0           sin(alp)           cos(alp)          d;
                0                  0                 0          1];
end