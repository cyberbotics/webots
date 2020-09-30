function result = wb_radar_get_vertical_fov(tag)
% Usage: wb_radar_get_vertical_fov(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/radar">here</a>

result = calllib('libController', 'wb_radar_get_vertical_fov', tag);
