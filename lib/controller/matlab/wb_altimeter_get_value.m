function result = wb_altimeter_get_value(tag)
% Usage: wb_altimeter_get_value(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/altimeter">here</a>

result = calllib('libController', 'wb_altimeter_get_value', tag);
