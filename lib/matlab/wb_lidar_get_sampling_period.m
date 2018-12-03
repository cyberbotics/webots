function result = wb_lidar_get_sampling_period(tag)
% Usage: wb_lidar_get_sampling_period(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

result = calllib('libController', 'wb_lidar_get_sampling_period', tag);
