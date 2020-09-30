function result = wb_supervisor_virtual_reality_headset_get_position()
% Usage: wb_supervisor_virtual_reality_headset_get_position()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_virtual_reality_headset_get_position');
setdatatype(obj,'doublePtr', 1, 3);
result = get(obj, 'Value');
