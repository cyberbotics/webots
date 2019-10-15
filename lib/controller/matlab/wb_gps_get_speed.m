function result = wb_gps_get_speed(tag)
% Usage: wb_gps_get_speed(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/gps">here</a>

result = calllib('libController', 'wb_gps_get_speed', tag);
