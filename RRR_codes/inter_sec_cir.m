function [x1,y1,x2,y2] = inter_sec_cir(l1,l2,l3,x0,y0)
    % this function computes the two points of intersection of two circles
    a = l1; b = x0; c = y0; d = l2 + l3;
    
    A = 4*(c)^2 + 4*(b)^2;
    B = -4*b*(a^2 + b^2 + c^2 - d^2);
    C = (a^2 + b^2 + c^2 - d^2)^2 - 4*(a^2)*(c^2);
    
    x1 = (-B + sqrt((B^2) - 4*A*C))/(2*A);
    x2 = (-B - sqrt((B^2) - 4*A*C))/(2*A);
    y1 = ((a^2 + b^2 + c^2 - d^2) - 2*b*x1)/(2*c);
    y2 = ((a^2 + b^2 + c^2 - d^2) - 2*b*x2)/(2*c);
end