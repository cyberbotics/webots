function result = wb_supervisor_node_get_field(noderef, fieldname)
% Usage: wb_supervisor_node_get_field(noderef, fieldname)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_node_get_field', noderef, fieldname);
