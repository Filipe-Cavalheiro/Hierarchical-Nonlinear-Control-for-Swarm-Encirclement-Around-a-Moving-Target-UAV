function geo = geo_distances(points, R, sphere_center)
% Compute unique geodesic distances between all drone pairs
%
% OUTPUT:
%   geo_distances_all -> [nPairs x 1] vector
%
% Pair ordering follows:
%   nchoosek(1:n_drones,2)

    if nargin < 3 || isempty(sphere_center)
        sphere_center = [0;0;0];
    end

    n_drones = size(points, 2);
    geo = zeros(n_drones, n_drones);
    
    for i = 1:n_drones-1
        for j = i+1:n_drones
    
            a = points(:,i);
            b = points(:,j);
    
            cos_theta = dot(a - sphere_center, b - sphere_center) / (R^2);
            cos_theta = max(min(cos_theta,1),-1);
    
            d = R * acos(cos_theta);
    
            geo(i,j) = d;
            geo(j,i) = d;   % mirror it
        end
    end
end