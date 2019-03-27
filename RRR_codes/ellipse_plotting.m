function [] = ellipse_plotting(A, rBall)
  
  %plot error ellipse
  figure(1)
  [V, D] = eig(A);
  [b, ind1] = max(abs(diag(D)));
  [a, ind2] = min(abs(diag(D)));

  b = rBall/sqrt(b);                % scaling minor axis
  a = rBall/sqrt(a);                % scaling major axis
  
  vMax = V(:,ind2);                 % major axis direction
  vMin = V(:,ind1);                 % minor axis direction

  thMax = atan2(vMax(2), vMax(1));  % major axis angle with +x
  thMin = atan2(vMin(2), vMin(1));  % minor axis angle with +x
  
  t = 0:pi/50:2*pi;   
  pts = zeros(2,length(t));

for i = 1:length(t)
  pts(1,i) = a*cos(t(i))*cos(thMax) - b*sin(t(i))*sin(thMax);
  pts(2,i) = a*cos(t(i))*sin(thMax) + b*sin(t(i))*cos(thMax);
end

  %patch(pts(1,:),pts(2,:),rand(1, 3))
  %patch(pts(1,:),pts(2,:),[0.0,1.0,1.0])
  plot(pts(1,:),pts(2,:))
 
  grid on
  grid minor
  xlabel('x[m]')
  ylabel('y[m]')
  
  drawnow limitrate
end
