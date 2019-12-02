function wb_distance_sensor_enable(tag, sampling_period)
% Usage: wb_distance_sensor_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/distancesensor">here</a>

calllib('libController', 'wb_distance_sensor_enable', tag, sampling_period);
