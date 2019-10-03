function result = wb_device_get_name(tag)
% Usage: wb_device_get_name(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/device">here</a>

result = calllib('libController', 'wb_device_get_name', tag);
