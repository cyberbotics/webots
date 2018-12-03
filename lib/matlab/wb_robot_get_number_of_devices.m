function result = wb_robot_get_number_of_devices()
% Usage: wb_robot_get_number_of_devices()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/receiver">here</a>

result = calllib('libController', 'wb_robot_get_number_of_devices');
