function wb_supervisor_field_set_mf_rotation(fieldref, index, values)
% Usage: wb_supervisor_field_set_mf_rotation(fieldref, index, values)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

assert(numel(values) == 4, 'Invalid ''values'' argument: 1x4 or 4x1 array expected');
calllib('libController', 'wb_supervisor_field_set_mf_rotation', fieldref, index, values);
