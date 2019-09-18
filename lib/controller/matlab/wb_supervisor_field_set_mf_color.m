function wb_supervisor_field_set_mf_color(fieldref, index, values)
% Usage: wb_supervisor_field_set_mf_color(fieldref, index, values)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

assert(numel(values) == 3, 'Invalid ''values'' argument: 1x3 or 3x1 array expected');
calllib('libController', 'wb_supervisor_field_set_mf_color', fieldref, index, values);
