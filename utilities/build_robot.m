function [rbt] = build_robot()
    lnk = 0.050;
    ak = [0 0 0];                          
    alpk = [-pi/2 pi/2 0];
    dk = [0 0 lnk];
    rbt = [ak; alpk; dk];
    
    fprintf('DH parameters of robot being used: \n');
    ak = rbt(1, :);
    alpk = rbt(2, :);
    dk = rbt(3, :);
    lnk = dk(1, 3);
    
    fprintf('|   a_k   |   alpha_k |  d_k  | theta_k |\n');
    for ii = 1:length(ak)
        fprintf('|%2.6f | %2.6f | %2.6f| th%d|\n', rbt(1, ii), rbt(2, ii), rbt(3, ii), ii);
    end
    fprintf('\n');
end