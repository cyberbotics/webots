function wb_lidar_enable(tag, sampling_period)
% Usage: wb_lidar_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

calllib('libController', 'wb_lidar_enable', tag, sampling_period);
