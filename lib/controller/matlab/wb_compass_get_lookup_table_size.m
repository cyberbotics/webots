function wb_compass_get_lookup_table_size(tag, sampling_period)
% Usage: wb_compass_get_lookup_table_size(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/compass">here</a>

calllib('libController', 'wb_compass_get_lookup_table_size', tag, sampling_period);
