clc
clear
close all

% Best and worst solutions
sol_b = [0.3947 1.4679 -2.6165];
sol_w = [2.0227 -2.1509 0.6022];

solvec = load('solvec.mat');

% Errors
dq = 0.005*ones(3,1);

% Define robot
l1 = 0.5; l2 = 0.5; l3 = 0.25;

% Link lengths of the manipulator
L(1) = Link([0 0 l1 0]);
L(2) = Link([0 0 l2 0]);
L(3) = Link([0 0 l3 0]);

% Build the RRR planar robot in Toolboox
ThreeLink = SerialLink(L);
ThreeLink.name = 'Planar3R';

% for i = 1:2
%     if i == 1
%         Jsol = ThreeLink.jacob0(sol_b);
%     else
%         Jsol = ThreeLink.jacob0(sol_w);
%     end
%     Jrsol = Jsol([1,2,6],:);
%     A = inv(Jrsol(1:2, :)*Jrsol(1:2,:)');
%     rBall = norm(dq);
%     ellipse_plotting(A, rBall);
%     hold on;
% end

for i = 1:size(solvec.sol_vec, 3)
    for j = 1:size(solvec.sol_vec, 1)
        Jsol = ThreeLink.jacob0(solvec.sol_vec(j,:,i));
        Jrsol = Jsol([1,2,6],:);
        A = inv(Jrsol(1:2, :)*Jrsol(1:2,:)');
        rBall = norm(dq);
        ellipse_plotting(A, rBall);
        hold on;
    end
    drawnow;
%     fig_frame(i)=getframe(gcf);
    im = frame2im(getframe(1));
    [imind,cm] = rgb2ind(im,256);
    outfile = 'error_ellipses.gif';
    if i==1
        imwrite(imind,cm,outfile,'gif','DelayTime',0,'loopcount',inf);
    else
        imwrite(imind,cm,outfile,'gif','DelayTime',0,'writemode','append');
    end
end

