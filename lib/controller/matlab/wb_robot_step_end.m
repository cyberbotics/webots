function result = wb_robot_step_end()
% Usage: wb_robot_step_end()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

result = calllib('libController', 'wb_robot_step_end');
