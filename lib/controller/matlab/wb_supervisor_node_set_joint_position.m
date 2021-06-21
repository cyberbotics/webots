function result = wb_supervisor_node_set_joint_position(noderef, position, index)
% Usage: wb_supervisor_node_set_joint_position(noderef, position, index)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_node_set_joint_position', noderef, position, index);
