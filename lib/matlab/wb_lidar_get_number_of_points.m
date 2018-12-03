function result = wb_lidar_get_number_of_points(tag)
% Usage: wb_lidar_get_number_of_points(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

result = calllib('libController', 'wb_lidar_get_number_of_points', tag);
