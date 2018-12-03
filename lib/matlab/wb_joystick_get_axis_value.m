function result = wb_joystick_get_axis_value(axis)
% Usage: wb_joystick_get_axis_value(axis)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/joystick">here</a>

result = calllib('libController', 'wb_joystick_get_axis_value', axis);
