function Rad = update_radius(Rad, e_q , Param)
    %UPDATE_RADIUS Summary of this function goes here
    %   Rad - radius of sphere
    %   e_q - Error value of q position
    %   chaser - chaser drone
    %   target - target drone
    %   Param - Parameters
    % ---------- Radius dynamics (purely scalar) ----------

    % V2 position (from error function)
    Rad_dot = Param.k_rad * ...
        exp(-(norm(e_q))) * ...
        (Param.sphere_radius - Rad);

    Rad = Rad + Param.dTi * Rad_dot;
end

