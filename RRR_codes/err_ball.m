clc;
clear all;
close all;

dTh = 0.005*ones(3,1);
rBall = norm(dTh);

% plot joint space error ball
figure (1)
[x,y,z] = plot_sphere_mod([0 0 0],rBall);
grid on
xlabel('x[rad]')
ylabel('y[rad]')
zlabel('z[rad]')
title('Error Bound in Joint Space')
view(42,51)

print -depsc errjs.eps

% process the points on the sphere
Xs = [x(:) y(:) z(:)]';

% plot tool space error ellipsoid
Jrsol = [-1.0303   -0.5880   -0.1277;
          0.1768    0.4101    0.2149;
          1.0000    1.0000    1.0000];
 
Elm = Jrsol*Jrsol';
StoE = sqrtm(Elm);

Xe = StoE*Xs;

% reshape the points on the ellipsoid
% put back to mesh format
Pxe = reshape(Xe(1,:), size(x));
Pye = reshape(Xe(2,:), size(y));
Pze = reshape(Xe(3,:), size(z));

figure(2)
%mesh(Pxe, Pye, zeros(size(z)),'FaceLighting','gouraud','LineWidth',0.3)
surf(Pxe, Pye, zeros(size(z)),'edgecolor','none')
%set(gcf,'units','points','position',[10,10,250,250])
grid on
xlabel('x[m]')
ylabel('y[m]')
zlabel('\theta[rad]')
title('Error Bound in Tool Space')

print -depsc errees1.eps

figure(3)
mesh(Pxe, Pye, Pze,'FaceLighting','gouraud','LineWidth',0.3)
print -depsc errees2.eps