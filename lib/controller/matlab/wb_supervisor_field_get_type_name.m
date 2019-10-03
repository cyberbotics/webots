function result = wb_supervisor_field_get_type_name(fieldref)
% Usage: wb_supervisor_field_get_type_name(fieldref)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_field_get_type_name', fieldref);
