function result = wb_robot_wwi_receive_text()
% Usage: wb_robot_wwi_receive_text()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

result = calllib('libController', 'wb_robot_wwi_receive_text');
