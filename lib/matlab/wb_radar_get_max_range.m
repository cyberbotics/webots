function result = wb_radar_get_max_range(tag)
% Usage: wb_radar_get_max_range(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/radar">here</a>

result = calllib('libController', 'wb_radar_get_max_range', tag);
