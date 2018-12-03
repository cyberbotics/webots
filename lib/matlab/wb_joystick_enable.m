function wb_joystick_enable(sampling_period)
% Usage: wb_joystick_enable(sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/joystick">here</a>

calllib('libController', 'wb_joystick_enable', sampling_period);
