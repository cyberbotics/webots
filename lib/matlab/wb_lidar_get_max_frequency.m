function result = wb_lidar_get_max_frequency(tag)
% Usage: wb_lidar_get_max_frequency(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

result = calllib('libController', 'wb_lidar_get_max_frequency', tag);
