clc
clear all
close all

% Ellipse matrix normal and symmetric
%==========================================
Amat = [0.3742 -0.2374;
       -0.2374  0.2683];

% Define the direction of error reduction
%===========================================
% a1 = 0.001513;
% b1 = 0.001888;
a1 = -1.5;
b1 = 1;
c1 = 0;

% Tangent points on the ellipse
%===============================
[V, D] = eig(Amat);
[a, ind1] = max(abs(diag(D)));
[b, ind2] = min(abs(diag(D)));

vMax = V(:,ind1);
vMin = V(:,ind2);

thMax = atan2(vMax(2), vMax(1));
thMin = atan2(vMin(2), vMin(1));

A = (cos(thMax)/a)^2 + (sin(thMax)/b)^2;
B = sin(thMax)*cos(thMax)*((1/a^2)-(1/b^2));
C = (sin(thMax)/a)^2 + (cos(thMax)/b)^2;
%D = -(a1*B+b1*C)/(a1*A+b1*B);
D = -(a1*B+b1*C)/(b1*B+a1*A);

A_tildae = A*(D^2) + 2*B*D + C;

y1 = sqrt(1/A_tildae);
y2 = -y1;
x1 = D*y1;
x2 = -x1;

display(['The points on the ellipse are : '])
fprintf('The first point on the ellipse is : (%f,%f)\n', x1, y1)
fprintf('The second point on the ellipse is : (%f,%f)\n\n', x2, y2)

% Point of intersection between tangent and line
% check if the point is a valid point
% m1*m2 = -1
m1 = -(A*x1+B*y1)/(B*x1+C*y1);
c1_tilde = y1 - m1*x1;
xcut = -(b1*c1_tilde)/(a1+b1*m1);
ycut = m1*xcut + c1_tilde;
m_tng = (ycut-y1)/(xcut-x1);
m_line = (ycut-0)/(xcut-0);
display(['Check if the tangent line and given line are perpendicular'])
if (m_tng*m_line + 1 <= 1e-10)
    fprintf('Yes\n');
else
    fprintf('No\n')
end

% Draw the ellipse and points at which the tangents are drawn
%==============================================================
th = 0:.1:2*pi;

for i=1:length(th)
    u(i) = a*cos(th(i));
    v(i) = b*sin(th(i));
    
    x(i) = u(i)*cos(thMax)-v(i)*sin(thMax);
    y(i) = u(i)*sin(thMax)+v(i)*cos(thMax);
end

% patch(x,y)
patch(x,y,[0.9 0.9 0.9])
grid on
xlabel('x')
ylabel('y')
title('Error bound in a given direction')
hold on
plot(x1,y1,'*',x2,y2,'*')
plot(xcut,ycut,'r*')
plot([xcut x1],[ycut y1],'k')

k = 1;
m = -(a1/b1);
for i = -20:20
    x_line(k) = 0.01*i;
    y_line(k) = m*x_line(k);
    k = k+1;
end

plot(x_line,y_line)
text(x_line(38),y_line(38),'\rightarrow minimization line')
text(x1,y1,'\leftarrow tangent point')
text(x2,y2,'\leftarrow tangent point')
text(xcut,ycut,'\leftarrow point of intersection')

% Axes of the coordinate frames
%=================================
drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0 );    
x1 = [0 0.40];
y1 = [0 0];
drawArrow(x1,y1);
text(0.4,0,'X')
hold on
x2 = [0 0];
y2 = [0 0.40];
drawArrow(x2,y2)
text(0,0.4,'Y')
xlim([-0.5 0.5])
ylim([-0.5 0.5])
axis equal