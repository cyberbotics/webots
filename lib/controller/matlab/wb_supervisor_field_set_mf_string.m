function wb_supervisor_field_set_mf_string(fieldref, index, string)
% Usage: wb_supervisor_field_set_mf_string(fieldref, index, string)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_field_set_mf_string', fieldref, index, string);
