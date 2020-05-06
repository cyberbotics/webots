function wb_light_sensor_get_lookup_table_size(tag)
% Usage: wb_light_sensor_get_lookup_table_size(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lightsensor">here</a>

calllib('libController', 'wb_light_sensor_get_lookup_table_size', tag);
