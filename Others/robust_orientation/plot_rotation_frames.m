clc
clear
close all

% Get the desired and actual quaternion
qd = [0.0619,-0.1705,-0.7749,-0.6055];
qa = [0.0200,-0.1350,-0.7919,-0.5952];

% Position of end effector
x = 0.5303; y = 1.1881; z = 0.3291;

% Rotation matrix coresponding to q_d
R_d = quat2rotm(qd);
R_a = quat2rotm(qa);

% Plot quaternion
for j = 1:2
    if j == 1
        R = R_d;
        col = 'r';
    else
        R = R_a;
        col = 'b';
    end
    for i = 1:3
        u = R(i,1);
        v = R(i,2);
        w = R(i,3);
        quiver3(x,y,z,u,v,w,col);
        hold on;
    end
end

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

txt = '\leftarrow P';
text(x,y,z,txt)

