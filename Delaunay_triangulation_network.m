function [connections, Laplacian, degree_matrix] = Delaunay_triangulation_network(positions, verbose)
    % Delaunay_triangulation_network - Creates connections using Delaunay
    % triangulation
    % Returns adjacency matrix, Laplacian matrix, and degree matrix
    
    if nargin < 2 || isempty(verbose)
        verbose = false;
    end

    % remove singleton dimensions
    if ndims(positions) ~= 3
        tol = 1e-9;
        isConstant = max(abs(positions - positions(1,:)), [], 1) < tol;
        positions(:, isConstant) = [];
    end
    
    N_agents = size(positions, 1);
    
    % Step 1: Create adjacency matrix (connections)
    % Start with no connections
    A = false(N_agents, N_agents); 
        
    DT = delaunayTriangulation(positions);
    
    if verbose
        figure;
        triplot(DT);
        hold on;
        plot(positions(:,1), positions(:,2), 'r*');
        pause;
    end 

    edges = DT.edges; 

    A = false(N_agents, N_agents);

    for k = 1:size(edges,1)
        i = edges(k,1);
        j = edges(k,2);
        A(i,j) = true;
        A(j,i) = true; % bidirectional graph
    end    
    
    % Step 3: Compute degree matrix and Laplacian
    % Degree matrix D (diagonal)
    degrees = sum(A, 2);
    degree_matrix = diag(degrees);
    
    % Laplacian matrix L = D - A
    % Convert A to double for Laplacian calculation
    A_double = double(A);
    Laplacian = degree_matrix - A_double;
    
    % Return adjacency matrix as connections (for backward compatibility)
    connections = A;
end