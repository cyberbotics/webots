function result = wb_supervisor_virtual_reality_headset_get_orientation()
% Usage: wb_supervisor_virtual_reality_headset_get_orientation()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_virtual_reality_headset_get_orientation');
setdatatype(obj,'doublePtr', 1, 9);
result = get(obj, 'Value');
