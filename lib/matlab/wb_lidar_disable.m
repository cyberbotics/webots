function wb_lidar_disable(tag)
% Usage: wb_lidar_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

calllib('libController', 'wb_lidar_disable', tag);
