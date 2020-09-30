function result = wb_compass_get_sampling_period(tag)
% Usage: wb_compass_get_sampling_period(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/compass">here</a>

result = calllib('libController', 'wb_compass_get_sampling_period', tag);
