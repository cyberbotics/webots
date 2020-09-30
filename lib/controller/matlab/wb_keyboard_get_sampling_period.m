function result = wb_keyboard_get_sampling_period()
% Usage: wb_keyboard_get_sampling_period()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/keyboard">here</a>

result = calllib('libController', 'wb_keyboard_get_sampling_period');
