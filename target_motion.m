function [target, c_sphere] = target_motion(target, p_d, Param)
%TARGET_MOTION Propagates target drone one timestep
%
% Inputs:
%   target   - struct with fields x (18x1) and xiep (3x1)
%   p_d      - desired position (3x1)
%   Param    - parameter struct
%
% Outputs:
%   target   - updated target struct
%   c_sphere - target position (3x1)

    % Extract state
    pos = target.x(1:3);
    v   = target.x(4:6);
    R   = reshape(target.x(7:15),3,3);
    om  = target.x(16:18);

    % Desired quantities (simple hover / trajectory tracking)
    v_d    = zeros(3,1);
    a_d    = zeros(3,1);
    j_d    = zeros(3,1);
    psi_d  = 0;
    dpsi_d = 0;

    % Controller (reuse drone controller)
    [T, tau, e_p] = drone_mellinger_ctrl( ...
        pos, v, R, om, Param, ...
        p_d, psi_d, target.xiep, ...
        v_d, dpsi_d, a_d, j_d);

    % Integrate position error
    target.xiep = target.xiep + Param.dTi*e_p;

    % Drone dynamics
    [dot_p, dot_v, dot_R, dot_om] = ...
        drone_3dfull_dyn(v, R, om, T, tau, Param);

    % Discretization (same as your agents)
    pos_p = pos + Param.dTi*dot_p*0.5;
    v_p   = v   + Param.dTi*dot_v*0.5;
    R_p   = rot_integrate(R, om, Param.dTi);
    om_p  = om  + Param.dTi*dot_om;

    % Pack state
    target.x = [ ...
        pos_p;
        v_p;
        reshape(R_p,[],1);
        om_p ];

    % Output target position
    c_sphere = pos_p;
end
