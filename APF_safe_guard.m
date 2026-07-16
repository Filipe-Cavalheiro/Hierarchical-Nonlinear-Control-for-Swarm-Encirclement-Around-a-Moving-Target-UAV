function q = APF_safe_guard(iD,  q_positions, min_safe_dist, Param)
%AFP_SAFE_GUARD Summary of this function goes here
%   Detailed explanation goes here
% parse the position from cells to array
    
%Compute geodesic distances between all pairs
current_geo_distances = geo_distances(q_positions, 1, [0;0;0]);

% APF collision avoidance
distances = current_geo_distances(iD, :);
[dist_vals, sorted_idx] = sort(distances);
%dist_vals = sorted_dists(1:(Param.nD-1));

artificial_potential_vector = zeros(3,1);
for i = 1:Param.nD-1
    if dist_vals(i) > min_safe_dist || dist_vals(i) == 0
        continue;
    end
    A_point = q_positions(:, iD);
    B_point = q_positions(:, sorted_idx(i));

    nA = (A_point - [0;0;0]);
    nA = nA / norm(nA);

    AB = A_point - B_point;

    % Remove radial component
    AB_tangent = AB - dot(AB, nA) * nA;

    if norm(AB_tangent) < 1e-6
        continue;
    end

    AB_tangent = AB_tangent / norm(AB_tangent);

    repulsion_mag = (min_safe_dist - dist_vals(i))*Param.k_apf;

    artificial_potential_vector = ...
        artificial_potential_vector + repulsion_mag * AB_tangent;
end

% Clamp
if norm(artificial_potential_vector) > min_safe_dist
    artificial_potential_vector = ...
        artificial_potential_vector / norm(artificial_potential_vector) * min_safe_dist;
end

% Apply as displacement
q = q_positions(:,iD) + artificial_potential_vector;
end

