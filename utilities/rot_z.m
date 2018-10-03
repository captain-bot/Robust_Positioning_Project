% rotation about Z axis
function [rot_mat] = rot_z(angle)
    rot_mat = eye(4, 4);
    rot_mat(1, 1) = cos(angle);
    rot_mat(1, 2) = -sin(angle);
    rot_mat(2, 1) = -rot_mat(1, 2);
    rot_mat(2, 2) = rot_mat(1, 1);
end
