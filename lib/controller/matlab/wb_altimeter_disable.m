function wb_altimeter_disable(tag)
% Usage: wb_altimeter_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/altimeter">here</a>

calllib('libController', 'wb_altimeter_disable', tag);
