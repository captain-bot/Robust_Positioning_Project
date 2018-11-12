function [rbt] = build_baxter_robot()
    ak = [0.069 0 0.069 0 0.010 0 0];
    dk = [0.27035 0 0.36435 0 0.37429 0 0.254525];
    alpk = pi*[-1/2 1/2 -1/2 1/2 -1/2 1/2 0];
    rbt = [ak; alpk; dk];
    
    fprintf('DH parameters of robot being used: \n');
    ak = rbt(1, :);
    alpk = rbt(2, :);
    dk = rbt(3, :);
    
    fprintf('|   a_k   |   alpha_k |  d_k  | theta_k |\n');
    for ii = 1:length(ak)
        fprintf('|%2.6f | %2.6f | %2.6f| th%d|\n', rbt(1, ii), rbt(2, ii), rbt(3, ii), ii);
    end
    fprintf('\n');
end