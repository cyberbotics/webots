function wb_inertial_unit_enable(tag, sampling_period)
% Usage: wb_inertial_unit_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/inertialunit">here</a>

calllib('libController', 'wb_inertial_unit_enable', tag, sampling_period);
