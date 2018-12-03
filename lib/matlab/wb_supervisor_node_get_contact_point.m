function result = wb_supervisor_node_get_contact_point(noderef, index)
% Usage: wb_supervisor_node_get_contact_point(noderef, index)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_node_get_contact_point', noderef, index);
setdatatype(obj,'doublePtr', 1, 3);
result = get(obj, 'Value');
