function result = wb_inertial_unit_get_roll_pitch_yaw(tag)
% Usage: wb_inertial_unit_get_roll_pitch_yaw(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/inertialunit">here</a>

obj = calllib('libController', 'wb_inertial_unit_get_roll_pitch_yaw', tag);
setdatatype(obj,'doublePtr', 1, 3);
result = get(obj, 'Value');
