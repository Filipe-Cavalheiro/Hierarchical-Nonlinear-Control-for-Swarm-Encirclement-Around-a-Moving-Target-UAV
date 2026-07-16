function [ref] = target_ref(ref_target_traj, target_pos)
%TARGET_REF Computes the next reference point along a trajectory
% based on the current target position.
%
% Input:
%   - target_pos (3x1): current target position
% Output:
%   - ref (3x1): desired next reference position

    persistent prev_idx
    N = size(ref_target_traj, 2);

    % Initialize index if first run
    if isempty(prev_idx)
        prev_idx = 1;
    end

    % Search only in a forward window to avoid jumping backwards
    window_size = 20; % tune this depending on speed
    idx_range = mod((prev_idx:prev_idx+window_size-1)-1, N) + 1;

    % Compute distances (Euclidean norm)
    distances = vecnorm(target_pos - ref_target_traj(:, idx_range), 2, 1);

    % Find closest point in the window
    [~, local_min_idx] = min(distances);
    closest_idx = idx_range(local_min_idx);

    % Move one step ahead (wrap-around)
    next_idx = mod(closest_idx, N) + 1;

    % Update memory
    prev_idx = closest_idx;

    % Output reference
    ref = ref_target_traj(:, next_idx);
end