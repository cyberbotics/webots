function wb_inertial_unit_get_lookup_table_size(tag)
% Usage: wb_inertial_unit_get_lookup_table_size(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/inertialunit">here</a>

calllib('libController', 'wb_inertial_unit_get_lookup_table_size', tag);
