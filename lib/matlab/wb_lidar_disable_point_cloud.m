function wb_lidar_disable_point_cloud(tag)
% Usage: wb_lidar_disable_point_cloud(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

calllib('libController', 'wb_lidar_disable_point_cloud', tag);
