clc
clear
close all

rx = 2;
ry = 1;
rz = 2/3;

phi = pi/4;
beta = pi/4;

x = rx*cos(phi)*cos(beta);
y = ry*cos(phi)*sin(beta);
z = rz*sin(phi);

a = 1;
b = 2;
c = 3;
d = 2;

M1 = 1;
M2 = 0.5;
M3 = 0.4714;

B1 = 0;
B2 = 0;
B3 = 0;

A = (M1/(2*a))^2 + (M2/(2*b))^2 + (M3/(2*c))^2;
t = d/sqrt(A);

x0 = (M1/(2*a^2))*t;
y0 = (M2/(2*b^2))*t;
z0 = (M3/(2*c^2))*t;
display(['The points on the ellipsoid at which the tangents are drawn :'])
fprintf('The first point is : (%f,%f,%f)\n',x0,y0,z0)
fprintf('The second point is : (%f,%f,%f)\n\n',-x0,-y0,-z0)

display(['Check if (x0,y0,z0) lies on the ellipsoid : '])
if (a*x0)^2 + (b*y0)^2 + (c*z0)^2 == d^2
    fprintf('Yes\n\n')
else
    fprintf('No\n\n')
end

B = M1*(a^2)*x0 + M2*(b^2)*y0 + M3*(c^2)*z0;
C = (a^2)*x0*(x0-B1)+(b^2)*y0*(y0-B2)+(c^2)*z0*(z0-B3);
t_tilde = C/B;

x1 = B1 + t_tilde*M1;
y1 = B2 + t_tilde*M2;
z1 = B3 + t_tilde*M3;

display(['Point of intersection between tangent plane and the directional line'])
fprintf('(x1,y1,z1) = (%f,%f,%f)\n\n',x1,y1,z1)

% Draw the ellipsoid
ellipsoid(0,0,0,rx,ry,rz)
xlabel('x')
ylabel('y')
zlabel('z')
hold on
plot3(x0,y0,z0,'k*')
plot3(x1,y1,z1,'b*')
v1=[x0 y0 z0];
v2=[x1 y1 z1];
v=[v2;v1];
plot3(v(:,1),v(:,2),v(:,3),'r')

display(['Check the orthonormality of the points : '])
if (M1*(x1-x0)+M2*(y1-y0)+M3*(z1-z0) <= 1e-10)
    fprintf('Yes\n\n')
end

for i = 1:100
    x(i) = B1+0.03*i*M1;
    y(i) = B2+0.03*i*M2;
    z(i) = B3+0.03*i*M3;
end

plot3(x,y,z)
text(x0,y0,z0,'\leftarrow (x0,y0,z0)')
text(x1,y1,z1,'\leftarrow (x1,y1,z1)')
title('Directional error reduction in Spatial Case')