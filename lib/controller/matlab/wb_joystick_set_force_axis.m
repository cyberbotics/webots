function wb_joystick_set_force_axis(axis)
% Usage: wb_joystick_set_force_axis(axis)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/joystick">here</a>

calllib('libController', 'wb_joystick_set_force_axis', axis);
