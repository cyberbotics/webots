function result = wb_led_get(tag)
% Usage: wb_led_get(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/led">here</a>

result = calllib('libController', 'wb_led_get', tag);
