function wb_supervisor_node_remove(noderef)
% Usage: wb_supervisor_node_remove(noderef)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_node_remove', noderef);
