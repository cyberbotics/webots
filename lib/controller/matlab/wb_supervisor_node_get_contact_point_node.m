function result = wb_supervisor_node_get_contact_point_node(noderef, index)
% Usage: wb_supervisor_node_get_contact_point_node(noderef, index)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_node_get_contact_point_node', noderef, index);
