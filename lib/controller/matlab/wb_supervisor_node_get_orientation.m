function result = wb_supervisor_node_get_orientation(noderef)
% Usage: wb_supervisor_node_get_orientation(noderef)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_node_get_orientation', noderef);
setdatatype(obj,'doublePtr', 3, 3);
result = get(obj, 'Value');
