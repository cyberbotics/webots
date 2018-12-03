function result = wb_joystick_get_number_of_povs()
% Usage: wb_joystick_get_number_of_povs()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/joystick">here</a>

result = calllib('libController', 'wb_joystick_get_number_of_povs');
