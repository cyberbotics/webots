function wb_supervisor_field_insert_mf_int32(fieldref, index, value)
% Usage: wb_supervisor_field_insert_mf_int32(fieldref, index, value)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_field_insert_mf_int32', fieldref, index, value);
