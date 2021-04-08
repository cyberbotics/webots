function result = wb_supervisor_node_get_pose(noderef, noderef_from)
% Usage: wb_supervisor_node_get_pose(noderef, noderef_from)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

if nargin < 2
    nullpointer = libpointer;
    obj = calllib('libController', 'wb_supervisor_node_get_pose', noderef, nullpointer);
else
    obj = calllib('libController', 'wb_supervisor_node_get_pose', noderef, noderef_from);
end
setdatatype(obj,'doublePtr', 4, 4);
result = get(obj, 'Value');
