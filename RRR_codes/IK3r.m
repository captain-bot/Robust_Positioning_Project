clc
clear all;
close all;

% Link lengths
l1 = 0.5; l2 = 0.5; l3 = 0.25;

% Given Configuration
x = 0.7156; y = 0.9309; phi = pi*(1/6+1/6+1/8);

% Errors in Joints Angles
dq = 0.005*ones(3,1);

%% Start of IK routine
X = x - l3*cos(phi);
Y = y - l3*sin(phi);

th2_1 = acos((X^2+Y^2-l1^2-l2^2)/(2*l1*l2));
th2_2 = -acos((X^2+Y^2-l1^2-l2^2)/(2*l1*l2));

k2_1 = l2*sin(th2_1);
k2_2 = l2*sin(th2_2);
k1_1 = l1 + l2*cos(th2_1);
k1_2 = l1 + l2*cos(th2_1);

th1_1 = atan2(Y, X) - atan2(k2_1, k1_1);
th1_2 = atan2(Y, X) - atan2(k2_2, k1_2);

th3_1 = phi - th1_1 - th2_1;
th3_2 = phi - th1_2 - th2_2;

th_1 = [th1_1 th2_1 th3_1];
th_2 = [th1_2 th2_2 th3_2];
