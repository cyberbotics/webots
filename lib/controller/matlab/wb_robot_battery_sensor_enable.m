function wb_robot_battery_sensor_enable(sampling_period)
% Usage: wb_robot_battery_sensor_enable(sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

calllib('libController', 'wb_robot_battery_sensor_enable', sampling_period);
