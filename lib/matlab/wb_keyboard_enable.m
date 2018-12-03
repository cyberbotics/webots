function wb_keyboard_enable(sampling_period)
% Usage: wb_keyboard_enable(sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/keyboard">here</a>

calllib('libController', 'wb_keyboard_enable', sampling_period);
