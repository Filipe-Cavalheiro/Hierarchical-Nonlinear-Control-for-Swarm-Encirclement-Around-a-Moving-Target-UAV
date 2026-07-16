function animate_simulation(data, Param)
%ANIMATE_SIMULATION Summary of this function goes here
%   Detailed explanation goes here
    figure('units','normalized','outerposition',[0 0 1 1]);
    hold on; grid on; 
    axis equal; 
    view(3);

    % Graphics handles
    for iD = 1:Param.nD
        initial_pos{iD} = scatter3(NaN, NaN, NaN, ...
            50, data.colors(iD,:), 'x', 'LineWidth', 1, ...
            'HandleVisibility','off');
        outer_traj{iD} = plot3(NaN, NaN, NaN,'--', 'Color', data.colors(iD,:), 'LineWidth', 1);
        inner_traj{iD} = plot3(NaN, NaN, NaN,'-', 'Color', data.colors(iD,:), 'LineWidth', 1);
        hAgent{iD} = plot3(NaN, NaN, NaN, 'x', 'Color', data.colors(iD,:), 'MarkerSize', 10, 'MarkerFaceColor', data.colors(iD,:), 'LineWidth', 3);
        href_ref_final{iD} = scatter3(NaN, NaN, NaN, 50, data.colors(iD,:), 'd', 'LineWidth', 1);
    end
    hInit = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'x', 'LineWidth', 2);

    hdrone = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'x');

    hp_d = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'd', 'LineWidth', 2);

    hp_T = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'x', 'LineWidth', 2);
    hCenter = plot3(NaN,NaN,NaN,'k','LineWidth',1.5);

    gfx.initial_pos = initial_pos;
    gfx.outer_traj = outer_traj;
    gfx.inner_traj = inner_traj;
    gfx.hAgent = hAgent;
    gfx.href_ref_final = href_ref_final;
    gfx.hCenter = hCenter;
    gfx.hdrone = hdrone;
    gfx.hInit = hInit;
    gfx.hp_d = hp_d;
    gfx.hp_T = hp_T;

    % === Create a label ===
    hInit = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'x', 'LineWidth', 2);

    hdrone = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'x');

    hp_d = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'd', 'LineWidth', 2);

    hp_T = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'o', 'LineWidth', 2);

    lgd = legend([hInit, hdrone, hp_d, hp_T], ...
        {'Initial Positions', 'drone', ...
         '$p_i^\star$', '$p_T$'}, ...
        'Interpreter', 'latex');

    lgd.FontSize = 10;
    lgd.Interpreter = 'latex';
    lgd.Position = [0.5825    0.7677    0.0752    0.1113];  % [x y width height]
    lgd.AutoUpdate = 'off';
    % === End of create a label ===
     gfx.lgd = lgd;

     plot3(data.target_position(1,:), ...
         data.target_position(2,:), ...
         data.target_position(3,:), 'k--');

    for k = 1:Param.animation.step:Param.Nsim
        animation_cycle(k, data, gfx, Param);
    end
end

