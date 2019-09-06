clc
clear
close all

% link lengths
l1 = 0.5; l2 = 0.5; l3 = 0.25;
% Joint error vector
dq = 0.005*ones(3,1);
% number of discretized points
n = 40;
% point to reach to in SE(2)
xd = 0.5; yd = 0.5; % phi = pi*(1/3-1/3+1/4);
%xd = 0.1768; yd = 1.0303; phi = pi*(1/4+1/4+1/4);
%x = 0.1853; y = 1.1575; phi = pi*(1/3+1/4-1/6);
% creating points between 0 and 2*pi
ang = linspace(0,2*pi,100);

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
lamvec = zeros(2, 1);
k = 1;                                  % increament counter

for i = 2:1:length(th)
    x3 = l1*cos(th(i));
    y3 = l1*sin(th(i));
    plot(x3, y3, 'o')
    hold on
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
        [V1, D1] = eig(J1([1,2],:)*J1([1,2],:)');
        [V2, D2] = eig(J2([1,2],:)*J2([1,2],:)');
        lamvec(1,k) = max(diag(D1));
        lamvec(2,k) = max(diag(D2));
        sol_vec(:,:,k) = [sol1; sol2];
        k = k + 1;
    end
end

[M,I] = min(lamvec(:));
[M_max, I_max] = max(lamvec(:));

%I = I_max;                             % if we need to plot worst solution

if mod(I, 2) == 0
    sol = sol_vec(2, :, I/2);
else
    sol = sol_vec(1, :, (I+1)/2);
end

if mod(I_max, 2) == 0
    sol_worst = sol_vec(2, :, I_max/2);
else
    sol_worst = sol_vec(1, :, (I_max+1)/2);
end

% Maximum Error in Best and Worst Cases
error_best = sqrt(dq'*dq*M);
error_worst = sqrt(dq'*dq*M_max);
    
% error reduction along a given direction
fprintf('Best case maximum error\n'); disp(error_best);
fprintf('Worst case maximum error\n'); disp(error_worst);

%////////////////////////////////////////////////////////%
%//////////////// PLOTTING RESULTS //////////////////////%
%////////////////////////////////////////////////////////%
Jsol = ThreeLink.jacob0(sol);
Jrsol = Jsol([1,2,6],:);
% Solving forward kinematics to check position
ThreeLink.fkine(sol)
    
% Plot the minimum error configuration
x1 = zeros(1, length(ang)); y1 = zeros(1, length(ang)); 
x2 = zeros(1, length(ang)); y2 = zeros(1, length(ang));

% coordinates of the points
for i = 1:1:length(ang)
    x1(1, i) = l1*cos(ang(i)); 
    y1(1, i) = l1*sin(ang(i));   
    x2(1, i) = xd + (l2 + l3)*cos(ang(i)); 
    y2(1, i) = yd + (l2 + l3)*sin(ang(i));
end

figure(1)
plot(x1,y1,x2,y2,xd,yd,'*',0,0,'o')
grid on
axis equal
xlabel('x[m]')
ylabel('y[m]')
hold on

[p1_b,p2_b,p3_b] = get_pts(sol(1), sol(2), sol(3), l1, l2, l3);
[p1_w,p2_w,p3_w] = get_pts(sol_worst(1), sol_worst(2), sol_worst(3), l1, l2, l3);

PlotMinConfig(p1_b, p2_b, p3_b, xd, yd);
PlotMinConfig(p1_w, p2_w, p3_w, xd,yd);

% Uncomment if interested in all solution
% for i = 1:32
%     for j = 1:2
%         sol = sol_vec(j, :, i);
%         [p1,p2,p3] = get_pts(sol(1), sol(2), sol(3), l1, l2, l3);
%         PlotMinConfig(p1,p2,p3,xd,yd);
%     end
% end

% draw error bounds
A = inv(Jrsol(1:2, :)*Jrsol(1:2,:)');
rBall = norm(dq);
draw_err(A, rBall);