clc
clear all
close all

a = 0.0065;
b = 0.0024;

t = 0:pi/50:2*pi;
th = atan2(0.6254,-0.7803);

pts = zeros(2,length(t));

for i = 1:length(t)
  pts(1,i) = a*cos(t(i))*cos(th) - b*sin(t(i))*sin(th);
  pts(2,i) = a*cos(t(i))*sin(th) + b*sin(t(i))*cos(th);
end

patch(pts(1,:),pts(2,:),[0.9,0.9,0.9])