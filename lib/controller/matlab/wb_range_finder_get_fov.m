function result = wb_range_finder_get_fov(tag)
% Usage: wb_range_finder_get_fov(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/rangefinder">here</a>

result = calllib('libController', 'wb_range_finder_get_fov', tag);
