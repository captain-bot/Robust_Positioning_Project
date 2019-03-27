function [wrst_included_ang] = check_ellipse_constraint(tval)
    global quat_des;

    tol = 1e-7;
    max_val = 1; min_val = tval;    
    if quat_des(4) ~= 0
        % Generate q1 and q2
        grid_range = [-1,1];    
        q1_array = grid_range(1):0.005:grid_range(2);
        q2_array = q1_array;
    end

    while (max_val-min_val) > tol
        avg_val = (max_val+min_val)/2;
        q_feas_array = [];
        ellp_sat = 0;
        % Generate full quaternion
         for i = 1:length(q1_array)
             for j = 1:length(q2_array)
                if q1_array(i)^2 + q2_array(j)^2 < 1
                    q_feas = comp_q(q1_array(i),q2_array(j),avg_val);
                    if ~isempty(q_feas)
                        q_feas_array = [q_feas_array,q_feas];
                    end
                end
            end
        end
        % Check Ellipsoid Constraint
        for i = 1:size(q_feas_array,2)
            if check_ellp(q_feas_array(:,i)) == 1
                max_val = avg_val;
                ellp_sat = 1;
                fprintf('Ellipse constraint satisfied\n');
                fprintf('q* = '); disp(q_feas_array(:,i)');
                fprintf('cos_th = %2.6f\n',avg_val);
                fprintf('q*qd^T = %2.6f\n',q_feas_array(:,i)'*quat_des);
                fprintf('Included ang (deg): %2.6f\n',acosd(q_feas_array(:,i)'*quat_des));
                fprintf('-----------------------------------------\n');
                wrst_included_ang = acosd(q_feas_array(:,i)'*quat_des);
                break;
            end
        end
        if ellp_sat == 0
            min_val = avg_val;
        end
    end
end

function [q] = comp_q(q1,q2,val)
    global quat_des;
    
    % Evaluate coefficients
    A = (val-quat_des(1)*q1-quat_des(2)*q2)/quat_des(end);
    B = quat_des(end-1)/quat_des(end);
    C = 1 - q1^2 - q2^2;
    D = 1 + B^2;
    E = -2*A*B;
    F = A^2-C;
    
    kernel_val = E^2-4*D*F;
    if kernel_val > 0
        % Find q3 s
        q3_1 = (-E + sqrt(kernel_val))/(2*D);
        q3_2 = (-E - sqrt(kernel_val))/(2*D);

        % Find q4 s
        q4_1 = A - B*q3_1;
        q4_2 = A - B*q3_2;

        % Construct quaternions
        q1_feas = [q1;q2;q3_1;q4_1];
        q2_feas = [q1;q2;q3_2;q4_2];

        q = [q1_feas,q2_feas];
    else
        q = [];
    end
end

function [bool_val] = check_ellp(q)
    global quat_des; global allJacs; global H_qd; global coeff_confi;

    ellp_constraint = (q-quat_des)'*H_qd'*inv(allJacs(4:end,:)*allJacs(4:end,:)')*H_qd*(q-quat_des);
    if ellp_constraint <= coeff_confi/4         % < 1e-7
        bool_val = 1;
    else
        bool_val = 0;
    end
end