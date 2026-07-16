function plot_graphs(data, Param)
%PLOT_GRAPHS Summary of this function goes here
%   Detailed explanation goes here
    figure('units','normalized','outerposition',[0 0 1 1]);
    hold on; grid on; 
    axis equal; 
    view(3);

    xlabel('x'); ylabel('y'); zlabel('z');

    % Graphics handles
    for iD = 1:Param.nD
        initial_pos{iD} = scatter3(NaN, NaN, NaN, 50, data.colors(iD,:), 'x', 'LineWidth', 1, 'HandleVisibility','off');
        outer_traj{iD} = plot3(NaN, NaN, NaN,'--', 'Color', data.colors(iD,:), 'LineWidth', 1);
        inner_traj{iD} = plot3(NaN, NaN, NaN,'-', 'Color', data.colors(iD,:), 'LineWidth', 1);
        hAgent{iD} = plot3(NaN, NaN, NaN, 'x', 'Color', data.colors(iD,:), 'MarkerSize', 10, 'MarkerFaceColor', data.colors(iD,:), 'LineWidth', 3);
        href_ref_final{iD} = scatter3(NaN, NaN, NaN, 50, 'k', 'd', 'LineWidth', 1);
    end
    hCenter = plot3(NaN,NaN,NaN,'k','LineWidth',1.5);

    gfx.initial_pos = initial_pos;
    gfx.outer_traj = outer_traj;
    gfx.inner_traj = inner_traj;
    gfx.hAgent = hAgent;
    gfx.href_ref_final = href_ref_final;
    gfx.hCenter = hCenter;

    % === Create a label ===
    gfx.hInit = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'x', 'LineWidth', 2);

    gfx.hdrone = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'x');

    gfx.hp_d = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'd', 'LineWidth', 2);

    gfx.hp_T = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'o', 'LineWidth', 2);

    lgd = legend([gfx.hInit, gfx.hdrone, gfx.hp_d, gfx.hp_T], ...
        {'Initial Positions', 'drone', ...
         '$p_i^\star$', '$p_T$'}, ...
        'Interpreter', 'latex');

    lgd.FontSize = 10; 
    lgd.Interpreter = 'latex';
    lgd.Position = [0.5825    0.7677    0.0752    0.1113];  % [x y width height]
    lgd.AutoUpdate = 'off';
    gfx.lgd = lgd;
    % === End of create a label ===

    % timesteps to highlight
    steps = 3000;
    steps(steps> Param.Nsim) = []; % remove values above simulation time
    [x_sphere, y_sphere, z_sphere] = sphere(40);

    surf(x_sphere*Param.sphere_radius + data.target_position(1, end),...
        y_sphere*Param.sphere_radius + data.target_position(2, end),...
        z_sphere*Param.sphere_radius + data.target_position(3, end), ...
        'EdgeColor', 'b', ...
        'EdgeAlpha', 0.05, ...
        'FaceAlpha', 0);

    Param.animation.trail_length = 0;
     axis off
    set(gca, 'Visible', 'off')
    set(gcf, 'Color', 'w');          % white background
    set(gca, 'Position', [0 0 1 1]);
    set(gca, 'LooseInset', max(get(gca,'TightInset'), 0));
    for k = steps
        % normal view print
        animation_cycle(k, data, gfx, Param);
        ax = gca;
        set_dynamic_camera(ax, data.target_position(:,k), [-1 -1 0.4], 60, 8);
        pause(1)
        print2pdf(sprintf("centralized_apf_%gt_%dDrones", k/100, Param.nD), 1);
    end
end

