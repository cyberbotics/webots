function wb_range_finder_disable(tag)
% Usage: wb_range_finder_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/rangefinder">here</a>

calllib('libController', 'wb_range_finder_disable', tag);
