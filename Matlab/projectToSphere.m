function projected_points = projectToSphere(points, r, center)
    % projectToSphere Projects 3D Cartesian points onto a spherical surface
    %
    % Usage:
    %   projected_points = projectToSphere(points)
    %   projected_points = projectToSphere(points, r)
    %   projected_points = projectToSphere(points, r, center)
    %
    % Inputs:
    %   points - Nx3 matrix of (x,y,z) Cartesian coordinates
    %   r      - Radius of the sphere (default is 1)
    %   center - 1x3 vector specifying sphere center (default is [0 0 0])
    %
    % Output:
    %   projected_points - Nx3 matrix of points projected onto the sphere

    if nargin < 2 || isempty(r)
        r = 1;
    end

    if nargin < 3 || isempty(center)
        center = [0 0 0];
    end

    % Ensure center is a row vector
    center = reshape(center, 3, 1);
    points = reshape(points, 3, []);

    % Shift points so sphere center is at the origin
    shifted_points = points - center;

    % Compute norms (row-wise)
    norms = sqrt(sum(shifted_points.^2, 1));
    norms(norms == 0) = eps;  % avoid division by zero

    % Project onto sphere and shift back
    projected_points = center + r * (shifted_points ./ norms);
end
