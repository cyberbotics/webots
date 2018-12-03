function wb_compass_disable(tag)
% Usage: wb_compass_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/compass">here</a>

calllib('libController', 'wb_compass_disable', tag);
