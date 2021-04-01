function result = wb_supervisor_node_get_relative_pose(noderef_from, noderef_to)
% Usage: wb_supervisor_node_get_relative_pose(noderef_from, noderef_to)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_node_get_relative_pose', noderef_from, noderef_to);
setdatatype(obj,'doublePtr', 4, 4);
result = get(obj, 'Value');
