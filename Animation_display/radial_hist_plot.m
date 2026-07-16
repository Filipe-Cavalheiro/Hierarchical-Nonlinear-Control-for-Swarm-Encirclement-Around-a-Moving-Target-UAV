function radial_hist_plot(data, Param)
%RADIAL_HIST_PLOT Plot radial distance history between all chasers and targetParam
%
% Inputs:
%   data.target_position : Target state history matrix
%   chaser      : Cell array containing chaser state histories
%   Nsim        : Number of simulation steps
%   Param       : Struct containing:
%                   - dTi : Time step
%                   - nD  : Number of chasers
%
% Example:
%   radial_hist_plot(data.target_position, chaser, Nsim, Param)

    figure;
    hold on;
    grid on;

    t = (0:Param.Nsim-1) * Param.dTi;

    % Transparency such that total transparency sums to 1
    alphaVal = 0.77;

    % Plot all chaser position differences
    for iD = 1:Param.nD

        posDiff = vecnorm( ...
            data.target_position(1:3,:) - data.chaser{iD}(1:3,1:end-1), ...
            2, 1);

        plot(t, posDiff, ...
            'LineWidth', 2, ...
            'Color', [data.colors(iD,:), alphaVal]);

    end

    xlabel('Time [s]');
    ylabel('Radial Distance [m]');

    lgd = legend('Radial distance (all agents to target)');

    lgd.FontSize = 24;
    lgd.Location = 'best';
    lgd.Interpreter = 'latex';

    hold off;
    print2pdf(sprintf("radial_hist_%d_agents", Param.nD), 1);
end