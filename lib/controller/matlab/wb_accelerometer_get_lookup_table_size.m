function wb_accelerometer_get_lookup_table_size(tag)
% Usage: wb_accelerometer_get_lookup_table_size(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/accelerometer">here</a>

calllib('libController', 'wb_accelerometer_get_lookup_table_size', tag);
