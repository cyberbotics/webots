function result = wb_lidar_get_vertical_fov(tag)
% Usage: wb_lidar_get_vertical_fov(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

result = calllib('libController', 'wb_lidar_get_vertical_fov', tag);
