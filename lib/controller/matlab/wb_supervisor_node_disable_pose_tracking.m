function result = wb_supervisor_node_disable_pose_tracking(node, from_node)
% Usage: wb_supervisor_node_disable_pose_tracking(node, from_node)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_node_disable_pose_tracking', node, from_node);
