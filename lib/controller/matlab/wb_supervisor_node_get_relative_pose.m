function result = wb_supervisor_node_get_pose(noderef)
% Usage: wb_supervisor_node_get_pose(noderef)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

if nargin < 2
    noderef_from = 0
end

obj = calllib('libController', 'wb_supervisor_node_get_pose', noderef, noderef_from);
setdatatype(obj,'doublePtr', 4, 4);
result = get(obj, 'Value');
