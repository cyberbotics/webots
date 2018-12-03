function wb_inertial_unit_disable(tag)
% Usage: wb_inertial_unit_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/inertialunit">here</a>

calllib('libController', 'wb_inertial_unit_disable', tag);
