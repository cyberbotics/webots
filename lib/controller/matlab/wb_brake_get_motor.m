function result = wb_brake_get_motor(tag)
% Usage: wb_brake_get_motor(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/brake">here</a>

result = calllib('libController', 'wb_brake_get_motor', tag);
