function wb_range_finder_enable(tag, sampling_period)
% Usage: wb_range_finder_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/rangefinder">here</a>

calllib('libController', 'wb_range_finder_enable', tag, sampling_period);
