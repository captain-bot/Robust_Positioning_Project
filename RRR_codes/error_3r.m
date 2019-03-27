clc;
clear;
close all;

% Change parameters
%openfig('err_ellipse.fig');
PCnt = 0;
a = 0.0065;
b = 0.0024;
thMax = 2.4659;
sol = [0.4598 1.2576 -2.7647];


% Do not touch the part after this line
% ================================================
A = (cos(thMax)/a)^2 + (sin(thMax)/b)^2;
B = sin(thMax)*cos(thMax)*((1/a^2)-(1/b^2));
C = (sin(thMax)/a)^2 + (cos(thMax)/b)^2;

% Link lengths of the manipulator
L(1) = Link([0 0 0.5 0]);
L(2) = Link([0 0 0.5 0]);
L(3) = Link([0 0 0.25 0]);

% Build the RRR planar robot in Toolboox
ThreeLink = SerialLink(L);
ThreeLink.name = 'Planar3R';

% Input : the joint angles
% qn = [pi/2 pi/2 pi/2];
% qn = [pi/6 pi/6 pi/8];
% qn = [1.0473   -0.5237    0.9163];
% qn = [1.3045   -0.5152   -0.4054];

qn = sol;
Tn = ThreeLink.fkine(qn);         % nominal pose
dq_mean = 0.005*ones(1,3);
k = 1;
%figure;

for k = 1 : 100
for i = 1:1000
   % Plot the robotTn
   % ThreeLink.plot(qn);
   % xlabel('X[m]')
   % ylabel('Y[m]')
   % zlabel('Z[m]')
   % title('3R Planar Manipulator')

   % Solve forward kinematics for joint vector
   
   dq = dq_mean.*randn(3,1)';
   Te = ThreeLink.fkine(qn + dq);    % erroneous pose

   % Compute the error in position
   % display('Error in position from forward kimatics solve with and without joint noise')
   del_pose = Te(1:3,4) - Tn(1:3,4);

   % Jacobian of the manipulator
   J1 = ThreeLink.jacob0(qn);
   % display('Error in position using dx = J(th)dth')
   del_t = J1*dq';
   
   % check if the point lies inside the ellipse
   x_ck = del_t(1);
   y_ck = del_t(2);
   
   evlt = ((cos(thMax)*x_ck+sin(thMax)*y_ck)/a)^2 + ((sin(thMax)*x_ck-cos(thMax)*y_ck)/b)^2;
   if (evlt - 1 <= 1e-4)
       PCnt = PCnt + 1;
   end
   
   % figure;
   %plot(del_pose(1,1),del_pose(2,1),'.')
   hold on
end
PCntr(k) = PCnt;
PCnt = 0;
k = k + 1;
end

mean(PCntr);