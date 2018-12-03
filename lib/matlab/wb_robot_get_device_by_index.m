function result = wb_robot_get_device_by_index(index)
% Usage: wb_robot_get_device_by_index(index)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/receiver">here</a>

result = calllib('libController', 'wb_robot_get_device_by_index', index);
