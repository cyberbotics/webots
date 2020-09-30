function result = wb_light_sensor_get_sampling_period(tag)
% Usage: wb_light_sensor_get_sampling_period(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lightsensor">here</a>

result = calllib('libController', 'wb_light_sensor_get_sampling_period', tag);
