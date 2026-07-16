function [q, e_q] = linear_motion_ref(q, q_d, Param)
    % ---------- Configuration error ----------
    e_q = skew(q_d) *q / sqrt(2*(1 + q' * q_d));

    % ---------- Reduced attitude kinematics ----------
    q_dot = -Param.k_geometric*skew(e_q)*q;
    q = q + Param.dTi * q_dot;
    q = q/norm(q);
end
