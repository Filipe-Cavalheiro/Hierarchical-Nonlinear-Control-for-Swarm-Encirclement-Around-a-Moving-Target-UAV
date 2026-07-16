function min_safe_dist = get_min_safe_distance(fib_ref_points, Param, verbose)
% MIN_SAFE_DISTANCE  Computes average neighbor (edge) distance on sphere

    if Param.nD < 3
        error('Not enough elements (Param.nD must be >= 3).');
    end

    if mod(Param.nD, 2) == 0
        error('Even number of elements is not allowed.');
    end

    if nargin < 3
        sphere_center = [0;0;0];
    end

    if nargin < 4
        verbose = false;
    end

    % Delaunay graph
    [connections, ~, ~] = Delaunay_triangulation_network(fib_ref_points');
    
    if sum(connections(:)) == 0
        connections = ones(Param.nD) - eye(Param.nD);
    end

    % Compute edge-based distances only
    edge_dists = [];

    for i = 1:Param.nD
    
        vals = geo_distances(fib_ref_points(:,connections(i,:) == 1), 1, sphere_center);
        vals = vals(vals ~= 0);
        
        if isempty(vals)
            min_val = NaN;   % or 0, depending on your use case
        else
            min_val = min(vals);
        end

        % indices of neighboring nodes
        edge_dists = [edge_dists; min_val];
    end

    % Average neighbor distance
    min_safe_dist = mean(edge_dists);

    if verbose
        fprintf('average_neighbor_distance: %.4f\n', min_safe_dist);
    end
end