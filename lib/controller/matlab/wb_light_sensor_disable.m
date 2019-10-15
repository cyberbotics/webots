function wb_light_sensor_disable(tag)
% Usage: wb_light_sensor_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lightsensor">here</a>

calllib('libController', 'wb_light_sensor_disable', tag);
