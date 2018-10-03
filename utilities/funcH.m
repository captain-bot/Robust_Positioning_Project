function [val] = funcH(q)
    % as per our latest derivation
    val = [q(1) q(4) -q(3) -q(2);
           -q(4) q(1) q(2) -q(3);
           q(3) -q(2) q(1) -q(4)];
end