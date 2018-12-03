function result = wb_robot_battery_sensor_get_value()
% Usage: wb_robot_battery_sensor_get_value()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

result = calllib('libController', 'wb_robot_battery_sensor_get_value');
