function result = wb_robot_battery_sensor_get_sampling_period()
% Usage: wb_robot_battery_sensor_get_sampling_period()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

result = calllib('libController', 'wb_robot_battery_sensor_get_sampling_period');
