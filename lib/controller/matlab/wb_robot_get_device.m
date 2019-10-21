function result = wb_robot_get_device(name)
% Usage: wb_robot_get_device(name)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

result = calllib('libController', 'wb_robot_get_device', name);
