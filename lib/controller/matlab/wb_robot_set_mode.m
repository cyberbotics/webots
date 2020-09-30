function wb_robot_set_mode(mode, arg)
% Usage: wb_robot_set_mode(mode, arg)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

calllib('libController', 'wb_robot_set_mode', mode, arg);
