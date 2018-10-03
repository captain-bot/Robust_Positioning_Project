function [ellp_stisfied, obj_val, vvec] = v_check(r, zp, scl, Lc, kvar, H, resl)
    avec = -zp;
    ellp_stisfied = 0;
    obj_val = 0;
    vvec = 0;
    
    % randomly initialize xp
    % then normalize
    zp = zp/norm(zp);

    x1 = 1; x2 = 1;
    x3 = -(zp(1)*x1+zp(2)*x2)/zp(3);
    xp = [x1; x2; x3]; xp = xp/norm(xp);


    yp = cross(zp, xp);
    rotmat = [xp yp zp];
    
    th = 0:resl:2*pi;
    pts = zeros(3, length(th));

    pts(1, :) = r*cos(th);
    pts(2, :) = r*sin(th);

    % transform the points into base frame from body frame
    base_pts = rotmat*pts;          % rotate
    base_pts = base_pts + scl * zp; % translate

    % check ellipsoid constraint
    for i = 1:size(base_pts, 2)
        if norm(Lc'*base_pts(:, i)) - 1 <= 1e-9
            ellp_stisfied = 1;
            objval = 1 + avec'*base_pts(:, i);
            if objval > obj_val
                obj_val = objval;
                vvec = base_pts(:, i);
            end
        end
    end
end