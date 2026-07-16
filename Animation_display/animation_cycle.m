function animation_cycle(k, data, gfx, Param)
    %ANIMATION_CYCLE Summary of this function goes here
    %   Detailed explanation goes here

    fprintf("Showing simulation interation: %04d\n", k);
    idx = max(1,k-Param.animation.trail_length):k;
    if idx(1) ~= 1 && data.removed_initial_pos == false
        data.removed_initial_pos = true;
    
        for iD = 1:Param.nD
            set(gfx.initial_pos{iD}, 'Visible', 'off');
        end
    
        % Remove only this item from legend
        idx = strcmp(string(gfx.lgd.String), 'Initial Positions');
        gfx.lgd.PlotChildren(idx) = [];  
    end
    
    set(gfx.hp_T, ...
        'XData', data.target_position(1,k), ...
        'YData', data.target_position(2,k), ...
        'ZData', data.target_position(3,k));

    for iD = 1:Param.nD
        set(gfx.href_ref_final{iD}, ...
            'XData', data.p_d(1, :) + data.target_position(1,k), ...
            'YData', data.p_d(2, :) + data.target_position(2,k), ...
            'ZData', data.p_d(3, :) + data.target_position(3,k));

        set(gfx.inner_traj{iD}, ...
            'XData', data.chaser{iD}(1,idx), ...
            'YData', data.chaser{iD}(2,idx), ...
            'ZData', data.chaser{iD}(3,idx));

        set(gfx.hAgent{iD}, ...
            'XData', data.chaser{iD}(1,k), ...
            'YData', data.chaser{iD}(2,k), ...
            'ZData', data.chaser{iD}(3,k));
    end
        
    set(gfx.hCenter, ...
        'XData', data.target_position(1,k), ...
        'YData', data.target_position(2,k), ...
        'ZData', data.target_position(3,k));     

    drawnow limitrate nocallbacks;

    if Param.animation.save_to_mp4
        frame = getframe(animation);
        writeVideo(v,frame);
    else
        % pause(0.01); % adjust speed of animation
    end
end

