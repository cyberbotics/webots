function wb_supervisor_node_set_velocity(noderef, velocity)
% Usage: wb_supervisor_node_set_velocity(noderef, velocity)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_node_set_velocity', noderef, velocity);
