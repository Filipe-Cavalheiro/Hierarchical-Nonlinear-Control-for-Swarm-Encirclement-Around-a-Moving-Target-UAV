function composed_fig(data, Param)
    figure;
    hold on; 
    grid on; 
    axis equal;
    view(3);

    target_position = data.target_position;
    chaser = data.chaser;
    p_d = data.p_d;
    
    % data.colors for chasers
    steps = [3000];
    steps(steps> Param.Nsim) = []; % remove values above simulation time
    
    % === Create a label ===
    hdrone = scatter3(NaN, NaN, NaN, ...
        50, 'k', 'x');

    htarget_path = plot3(NaN, NaN, NaN, 'k--');
    
    htarget_pos= scatter3(NaN, NaN, NaN, 'k', 'o');

    hpath = plot3(NaN, NaN, NaN, 'k');

    hfinal_pos = scatter3(NaN, NaN, NaN, 'k', 'd');

    lgd = legend([hdrone, hpath, htarget_pos, htarget_path, hfinal_pos], ...
        {'$p_i$' ,'$p_i$ path', '$p_T$', '$p_T$ path',...
         '$p^\star$'}, 'Interpreter','latex');

    lgd.FontSize = 10; 
    lgd.Interpreter = 'latex';
    lgd.Position = [0.5825    0.7677    0.0752    0.1113];  % [x y width height]
    lgd.AutoUpdate = 'off';
    % === End of create a label ===
    
    plot3(Param.ref_target_traj(1,:), Param.ref_target_traj(2,:), Param.ref_target_traj(3,:), 'k--');

    
    for iD = 1:Param.nD
        plot3(chaser{iD}(1,:), ...
              chaser{iD}(2,:), ...
              chaser{iD}(3,:), ...
              'Color', data.colors(iD,:));
    end
    
    
    for k = steps
        % --- target position at timestep k ---
        scatter3(target_position(1,k), ...
                 target_position(2,k), ...
                 target_position(3,k), ...
                 'k', ...
                 'o');

        % --- chaser positions at timestep k ---
        for iD = 1:Param.nD
            scatter3(p_d(1,iD) + data.target_position(1,k), ...
                     p_d(2,iD) + data.target_position(2,k), ...
                     p_d(3,iD) + data.target_position(3,k), ...
                     'Marker', 'd', ...
                     'MarkerEdgeColor', data.colors(iD,:));

            scatter3(chaser{iD}(1,k), ...
                     chaser{iD}(2,k), ...
                     chaser{iD}(3,k), ...
                     'x', 'MarkerEdgeColor', data.colors(iD,:));
        end
    end
    print2pdf(sprintf("composed_%d_agents", Param.nD), 1);
end

