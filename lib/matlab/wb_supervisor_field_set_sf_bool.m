function wb_supervisor_field_set_sf_bool(fieldref, value)
% Usage: wb_supervisor_field_set_sf_bool(fieldref, value)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_field_set_sf_bool', fieldref, value);
