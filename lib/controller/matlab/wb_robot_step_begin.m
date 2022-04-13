function result = wb_robot_step_begin(duration)
% Usage: wb_robot_step_begin(duration)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

result = calllib('libController', 'wb_robot_step_begin', duration);
