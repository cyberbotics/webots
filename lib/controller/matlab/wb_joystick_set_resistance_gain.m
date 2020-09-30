function wb_joystick_set_resistance_gain(gain)
% Usage: wb_joystick_set_resistance_gain(gain)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/joystick">here</a>

calllib('libController', 'wb_joystick_set_resistance_gain', gain);
