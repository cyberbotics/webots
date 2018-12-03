function result = wb_compass_get_values(tag)
% Usage: wb_compass_get_values(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/compass">here</a>

obj = calllib('libController', 'wb_compass_get_values', tag);
setdatatype(obj,'doublePtr', 1, 3);
result = get(obj, 'Value');
