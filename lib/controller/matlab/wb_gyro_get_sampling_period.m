function result = wb_gyro_get_sampling_period(tag)
% Usage: wb_gyro_get_sampling_period(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/gyro">here</a>

result = calllib('libController', 'wb_gyro_get_sampling_period', tag);
