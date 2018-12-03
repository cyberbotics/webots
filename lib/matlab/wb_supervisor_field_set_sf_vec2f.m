function wb_supervisor_field_set_sf_vec2f(fieldref, values)
% Usage: wb_supervisor_field_set_sf_vec2f(fieldref, values)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

assert(numel(values) == 2, 'Invalid ''values'' argument: 1x2 or 2x1 array expected');
calllib('libController', 'wb_supervisor_field_set_sf_vec2f', fieldref, values);
