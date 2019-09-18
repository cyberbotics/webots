function wb_robot_set_data(data)
% Usage: wb_robot_set_data(data)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

calllib('libController', 'wb_robot_set_data', data);
