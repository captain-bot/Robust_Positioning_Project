function[translation_mat] = translation(x, y, z)
    translation_mat = eye(4, 4);
    translation_mat(1, 4) = x;
    translation_mat(2, 4) = y;
    translation_mat(3, 4) = z;
end