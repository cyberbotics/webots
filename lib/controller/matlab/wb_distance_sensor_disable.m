function wb_distance_sensor_disable(tag)
% Usage: wb_distance_sensor_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/distancesensor">here</a>

calllib('libController', 'wb_distance_sensor_disable', tag);
