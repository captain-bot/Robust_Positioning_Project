function[overall_success] = MCSimulation(num_trial, mu, sig, num_sig, qd, qr, solIKs, rbt, rbt_base)
    fprintf('Monte-Carlo simulation: \n');    
    
    % DH aprameters of Spherical Joint
    ak = rbt(1, :);
    alpk = rbt(2, :);
    dk = rbt(3, :);
    lnk = dk(1, 3);
    
    % Start of Monte-Carlo simulation
    dot_prod_qr_qd = qd*qr;
    lower_count = 0;
    outlier_count = 0;
    outlier_lower_count = 0;
    for sample_num = 1:num_trial
        gen_err = normrnd(mu, sig, size(solIKs));
        greater_flag = 0;
        for jj = 1:length(gen_err)
            if gen_err(jj) < num_sig*sig
                greater_flag = 0;
            else
                greater_flag = 1;
                outlier_count = outlier_count + 1;
                break;
            end
        end
        actual_joint_sol = solIKs + gen_err;

        % Solve forward kinematics
        Tf = rbt_base;
        for i = 1:length(ak)
            Tf = Tf*lcl_trns(ak(1, i), alpk(1, i), dk(1, i), actual_joint_sol(1, i));
        end

        % Compute included angle between desired and achieved quaternions
        actual_quat1 = rotm2quat(Tf(1:3, 1:3));
        actual_quat2 = -rotm2quat(Tf(1:3, 1:3));
        
        dot_prod_qa_qd = max(qd*actual_quat1', qd*actual_quat2');
        included_angle = acosd(dot_prod_qa_qd);
        

        %if included_angle < worst_included_angle
        %if (dot_prod_q_qd > dot_prod_wrst_desired_quat) || (-dot_prod_q_qd > dot_prod_wrst_desired_quat)
        if (dot_prod_qa_qd > dot_prod_qr_qd)
            if greater_flag == 0
                lower_count = lower_count + 1;
            elseif greater_flag == 1
                outlier_lower_count = outlier_lower_count + 1;
            end
        end
    end
    
    fprintf('Number of samples considered: %d\n', num_trial);
    overall_success = ((lower_count + outlier_lower_count)/num_trial)*100;
    fprintf('Success rate (overall): %2.4f\n', overall_success);
    fprintf('Success rate (excluding outlier success): %2.4f\n', (lower_count/(num_trial - outlier_count))*100);
    fprintf('Total outliers: %d\n', outlier_count);
    fprintf('Outlier success: %d\n', outlier_lower_count);
    fprintf('--------------------------------------\n');
end
