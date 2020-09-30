function wb_touch_sensor_disable(tag)
% Usage: wb_touch_sensor_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/touchsensor">here</a>

calllib('libController', 'wb_touch_sensor_disable', tag);
