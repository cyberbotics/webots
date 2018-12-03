function wb_robot_wwi_send_text(text)
% Usage: wb_robot_wwi_send_text(text)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

calllib('libController', 'wb_robot_wwi_send_text', text);
