function result = wb_inertial_unit_get_quaternion(tag)
% Usage: wb_inertial_unit_get_quaternion(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/inertialunit">here</a>

obj = calllib('libController', 'wb_inertial_unit_get_quaternion', tag);
setdatatype(obj,'doublePtr', 1, 4);
result = get(obj, 'Value');
