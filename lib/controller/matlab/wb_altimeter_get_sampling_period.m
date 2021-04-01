function result = wb_altimeter_get_sampling_period(tag)
% Usage: wb_altimeter_get_sampling_period(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/altimeter">here</a>

result = calllib('libController', 'wb_altimeter_get_sampling_period', tag);
