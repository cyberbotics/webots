function wb_lidar_set_frequency(tag,frequency)
% Usage: wb_lidar_set_frequency(tag,frequency)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

calllib('libController', 'wb_lidar_set_frequency', tag,frequency);
