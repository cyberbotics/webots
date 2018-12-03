function result = wb_joystick_is_connected()
% Usage: wb_joystick_is_connected()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/joystick">here</a>

result = calllib('libController', 'wb_joystick_is_connected');
