function [] = draw_err(A, rBall)
  % plot joint space error ball
  figure(2)
  subplot(1, 2, 1)
  plot_sphere_mod([0 0 0],rBall);
  grid on
  xlabel('x[rad]')
  ylabel('y[rad]')
  zlabel('z[rad]')
  title('Error Bound in Joint Space')
  view(42,51)

%   print -depsc errjs.eps
  
  %plot error ellipse
  figure(2)
  subplot(1, 2, 2)
  [V, D] = eig(A);
  [a, ind1] = max(abs(diag(D)));
  [b, ind2] = min(abs(diag(D)));

  vMax = V(:,ind1);
  vMin = V(:,ind2);

  thMax = atan2(vMax(2), vMax(1));
  thMin = atan2(vMin(2), vMin(1));
  
  a = rBall/sqrt(a);    % scaling minor axis
  b = rBall/sqrt(b);    % sccaling major axis
  t = 0:pi/50:2*pi;   
  pts = zeros(2,length(t));

for i = 1:length(t)
  pts(1,i) = a*cos(t(i))*cos(thMax) - b*sin(t(i))*sin(thMax);
  pts(2,i) = a*cos(t(i))*sin(thMax) + b*sin(t(i))*cos(thMax);
end

  patch(pts(1,:),pts(2,:),[0.0,1.0,1.0])
  %plot(pts(1,:),pts(2,:))
 
  grid on
  grid minor
  xlabel('x[m]')
  ylabel('y[m]')
  %ylim([-6e-3, 6e-3])
  %ylim([-5e-3, 5e-3])
  %axis square
  %title({'Error Bound in End Effector Space';'                   '})
  %title('Error Bound in End Effector Space Mapped from Joint Space')
  
  %print -depsc errees.eps
end
