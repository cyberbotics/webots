function result = wb_joystick_get_pressed_button()
% Usage: wb_joystick_get_pressed_button()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/joystick">here</a>

result = calllib('libController', 'wb_joystick_get_pressed_button');
