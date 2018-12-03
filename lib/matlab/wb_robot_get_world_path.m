function result = wb_robot_get_world_path()
% Usage: wb_robot_get_world_path()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

result = calllib('libController', 'wb_robot_get_world_path');
