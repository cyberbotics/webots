function wb_supervisor_node_add_relative_force(noderef, force, origin)
% Usage: wb_supervisor_node_add_relative_force(noderef, force, origin)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_node_add_relative_force', noderef, force, origin);
