function plotcube(varargin)
% plotcube([X Y Z], [x0 y0 z0], alpha, color)

dims  = varargin{1};
origin = varargin{2};
alpha  = varargin{3};
color  = varargin{4};
linestyle  = varargin{5};

X = [0 1 1 0 0 1 1 0] * dims(1) + origin(1);
Y = [0 0 1 1 0 0 1 1] * dims(2) + origin(2);
Z = [0 0 0 0 1 1 1 1] * dims(3) + origin(3);

faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];

patch('Vertices',[X' Y' Z'], ...
      'Faces',faces, ...
      'FaceColor',color, ...
      'FaceAlpha',alpha, ...
      'EdgeColor','k', ...
      'LineStyle', linestyle, ...
      'LineWidth',1.2);
end
