function wb_gps_disable(tag)
% Usage: wb_gps_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/gps">here</a>

calllib('libController', 'wb_gps_disable', tag);
