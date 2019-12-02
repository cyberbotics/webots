function wb_position_sensor_disable(tag)
% Usage: wb_position_sensor_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/positionsensor">here</a>

calllib('libController', 'wb_position_sensor_disable', tag);
