function wb_touch_sensor_enable(tag, sampling_period)
% Usage: wb_touch_sensor_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/touchsensor">here</a>

calllib('libController', 'wb_touch_sensor_enable', tag, sampling_period);
