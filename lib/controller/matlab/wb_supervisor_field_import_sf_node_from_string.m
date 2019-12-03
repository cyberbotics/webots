function wb_supervisor_field_import_sf_node_from_string(fieldref, node_string)
% Usage: wb_supervisor_field_import_sf_node_from_string(fieldref, node_string)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_field_import_sf_node_from_string', fieldref, node_string);
