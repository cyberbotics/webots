function result = wb_supervisor_node_get_parent_node(noderef)
% Usage: wb_supervisor_node_get_parent_node(noderef)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_node_get_parent_node', noderef);
