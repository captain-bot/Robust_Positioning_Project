clc
clear all
close all

a = 0.01;
b = 0.03;
th = 0:.1:2*pi;

for i=1:length(th)
    u(i) = a*cos(th(i));
    v(i) = b*sin(th(i));
    
    x(i) = u(i)*cos(pi/4)+v(i)*sin(pi/4);
    y(i) = -u(i)*sin(pi/4)+v(i)*cos(pi/4);
end

patch(x,y,'yellow')
grid on
xlabel('x')
ylabel('y')
title('Error bound in a given direction')
hold on