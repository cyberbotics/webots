function wb_inertial_unit_get_lookup_table(tag, sampling_period)
% Usage: wb_inertial_unit_get_lookup_table(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/inertialunit">here</a>

size = calllib('libController', 'wb_inertial_unit_get_lookup_table_size', tag);
pointer = calllib('libController', 'wb_inertial_unit_get_lookup_table', tag);
setdatatype(pointer, 'doublePtr', 3 * size, 1);
result = get(pointer, 'Value');
