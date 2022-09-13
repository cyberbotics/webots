function points = wb_lidar_get_point_cloud(tag)
% Usage: wb_lidar_get_point_cloud(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

rate = calllib('libController', 'wb_lidar_get_sampling_period', tag);
point_cloud_enable = calllib('libController', 'wb_lidar_is_point_cloud_enabled', tag);
if (rate <= 0) || (point_cloud_enable == false)
    points = []
    if (rate <= 0)
        calllib('libController', 'wb_console_print', 'Warning: wb_lidar_get_point_cloud() called for a disabled lidar! Please call wb_lidar_enable() before.', 2);
    else
        calllib('libController', 'wb_console_print', 'Warning: wb_lidar_get_point_cloud() called for a lidar with point cloud disabled! Please call wb_lidar_enable_point_cloud() before.', 2);
    end
else
    size = calllib('libController', 'wb_lidar_get_number_of_points', tag);
    for n = 1:size
        obj = calllib('libController', 'wb_lidar_get_point', tag, n - 1);
        points(n) = obj.Value;
    end
end
