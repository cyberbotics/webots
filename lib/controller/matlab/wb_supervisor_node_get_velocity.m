function result = wb_supervisor_node_get_velocity(noderef)
% Usage: wb_supervisor_node_get_velocity(noderef)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_node_get_velocity', noderef);
setdatatype(obj,'doublePtr', 1, 6);
result = get(obj, 'Value');
