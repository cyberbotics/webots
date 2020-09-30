function wb_led_set(tag, value)
% Usage: wb_led_set(tag, value)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/led">here</a>

calllib('libController', 'wb_led_set', tag, value);
