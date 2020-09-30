function wb_radar_disable(tag)
% Usage: wb_radar_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/radar">here</a>

calllib('libController', 'wb_radar_disable', tag);
