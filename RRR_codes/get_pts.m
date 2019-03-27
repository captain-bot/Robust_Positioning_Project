function [p1,p2,p3] = get_pts(theta1,theta2,theta3, l1, l2, l3)
 % this function returns the end points of the joints of the 3R manipulator
 p1_x = l1*cos(theta1);
 p1_y = l1*sin(theta1);
 p1 = [p1_x p1_y];
 
 p2_x = p1_x + l2*cos(theta1+theta2);
 p2_y = p1_y + l2*sin(theta1+theta2);
 p2 = [p2_x p2_y];
 
 p3_x = p2_x + l3*cos(theta1 + theta2 + theta3);
 p3_y = p2_y + l3*sin(theta1 + theta2 + theta3);
 p3 = [p3_x p3_y];
 
end