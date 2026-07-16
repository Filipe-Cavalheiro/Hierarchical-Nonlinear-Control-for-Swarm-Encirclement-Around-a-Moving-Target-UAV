function set_dynamic_camera(ax, target_pos, offset_dir, distance, view_angle)
    offset_dir = offset_dir / norm(offset_dir); % normalize direction
    
    cam_target = target_pos(:)';
    cam_position = cam_target + distance * offset_dir;

    ax.CameraTarget = cam_target;
    ax.CameraPosition = cam_position;
    ax.CameraUpVector = [0 0 1];
    ax.CameraViewAngle = view_angle;
end