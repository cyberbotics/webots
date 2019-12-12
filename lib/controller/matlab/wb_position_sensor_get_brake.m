function result = wb_position_sensor_get_brake(tag)
% Usage: wb_position_sensor_get_brake(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/positionsensor">here</a>

result = calllib('libController', 'wb_position_sensor_get_brake', tag);
