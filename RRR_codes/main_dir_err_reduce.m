clc
clear
close all

% link lengths
l1 = 0.5; l2 = 0.5; l3 = 0.25;
% Joint error vector
dq = 0.005*ones(3,1);
cval = dq'*dq;
% number of discretized points
n = 40;
% point to reach to in SE(2)
xd = 0.5; yd = 0.5; % phi = pi*(1/3-1/3+1/4);

% creating points between 0 and 2*pi
ang = linspace(0,2*pi,100);

% direction of the line along which we want to reduce error
%v_line = [cos(pi/4); sin(pi/4)];  % must be a unit vector
v_line = [1; 0];

%/////////////////////////////////////////////////////////%
%/////////////////// DO NOT TOUCH ////////////////////////%
%/////////////////////////////////////////////////////////%

% Link lengths of the manipulator
L(1) = Link([0 0 l1 0]);
L(2) = Link([0 0 l2 0]);
L(3) = Link([0 0 l3 0]);

% Build the RRR planar robot in Toolboox
ThreeLink = SerialLink(L);
ThreeLink.name = 'Planar3R';

% find the cross section of the two circles
[x_1,y_1,x_2,y_2] = inter_sec_cir(l1,l2,l3,xd,yd);
th2(1) = atan2(y_1,x_1);
th2(2) = atan2(y_2,x_2);

% discretize the range of THETA
[th] = discretize(th2, l1, l2, l3, n, xd, yd);
Link_Lengths = [l2; l3];
sol_vec = zeros(2, 3, 1);
intrcpt_vec = zeros(2, 1);
k = 1;                                  % increament counter

for i = 2:1:length(th)
    x3 = l1*cos(th(i));
    y3 = l1*sin(th(i));
    
    % depending on the quadrant of the target point the following equations
    xc = xd - x3;
    yc = yd - y3;
    pose = [xc yc];
    [theta_1,  theta_2, val] = RR_inverse(Link_Lengths, pose);
    if val == 'N'
        continue;
    else
        theta21 = theta_1(1) - th(i);  % second variable in global frame
        theta22 = theta_2(1) - th(i);  % second variable in global frame
        sol1 = [th(i) theta21 theta_1(2)];
        sol2 = [th(i) theta22 theta_2(2)];
        J1 = ThreeLink.jacob0(sol1);
        J2 = ThreeLink.jacob0(sol2);
        
        intrcpt_vec(1,k) = intrcept_line(J1(1:2, :), cval, v_line);
        intrcpt_vec(2,k) = intrcept_line(J2(1:2, :), cval, v_line);
        
        sol_vec(:,:,k) = [sol1; sol2];
        k = k + 1;
    end
end

[M,I] = min(intrcpt_vec(:));
[M_max, I_max] = max(intrcpt_vec(:));

%I = I_max;                             % if we need to plot worst solution
for i = 1:2
    if i == 2
        I = I_max;
        M = M_max;
    end
    
    if mod(I, 2) == 0
        sol = sol_vec(2, :, I/2);
    else
        sol = sol_vec(1, :, (I+1)/2);
    end
    fprintf('Magnitude of error\n'); disp(M);
    fprintf('The joint solution is: \n'); disp(sol);
    
    % plot the ellipse
    Jsol = ThreeLink.jacob0(sol);
    Jrsol = Jsol([1,2,6],:);
    A = inv(Jrsol(1:2, :)*Jrsol(1:2,:)');
    rBall = norm(dq);
    draw_err(A, rBall);
    hold on;
    % ThreeLink.fkine(sol)
end

% draw the axes
quiver(0, 0, 0, 0.006, 0, 'k')
quiver(0, 0, 0.008, 0, 0, 'k')