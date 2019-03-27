function [th] = discretize(th2, l1, l2, l3, n, x, y)
     
     a = min(th2);
     b = max(th2);  
     th = zeros(1,n);
     
     % this function returns the discretized points on circle of radius l1
     %th = linspace(a,b,n);
     h = ((b-a)/n); % rate of decreament/increament
     for i = 1 : 1 : n
         th(i) = b - (h*(i - 1));
         if th(i) < -pi
             th(i) = pi + (th(i) + pi);
         end
     end

     % check if the midpoint of th lies inside the circle of radius l2 + l3
     xpt = l1*cos(th(n/2));
     ypt = l1*sin(th(n/2));
     
     % xpt, ypt expressed in the frame fixed at the target point
     xt = xpt - x;
     yt = ypt - y;
     
     % check if this lies inside the circle of radius l2+ l3
     check = (xt^2) + (yt^2) - (l2 + l3)^2;
     if check ==0 || check < 0
         return
     else
         th = zeros(1,n);
         h = ((2*pi - (a-b))/n);
         for i = 1: 1: n
             th(i) = a - (h)*(i - 1);
             %th(i)
             if th(i) < -pi
                 th(i) = pi + (th(i)+ pi);
                 if abs(th(i)) < abs(b)
                     return
                 end
             end
         end
     end    
end