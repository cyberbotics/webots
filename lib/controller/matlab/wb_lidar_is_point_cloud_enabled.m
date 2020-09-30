function result = wb_lidar_is_point_cloud_enabled(tag)
% Usage: wb_lidar_is_point_cloud_enabled(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

result = calllib('libController', 'wb_lidar_is_point_cloud_enabled', tag);
