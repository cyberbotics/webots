function wb_supervisor_field_import_sf_node(fieldref, filename)
% Usage: wb_supervisor_field_import_sf_node(fieldref, filename)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_field_import_sf_node', fieldref, filename);
