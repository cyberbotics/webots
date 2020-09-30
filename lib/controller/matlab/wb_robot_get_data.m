function result = wb_robot_get_data()
% Usage: wb_robot_get_data()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

result = calllib('libController', 'wb_robot_get_data');
