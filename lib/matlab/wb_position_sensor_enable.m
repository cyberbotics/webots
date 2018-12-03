function wb_position_sensor_enable(tag, sampling_period)
% Usage: wb_position_sensor_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/positionsensor">here</a>

calllib('libController', 'wb_position_sensor_enable', tag, sampling_period);
