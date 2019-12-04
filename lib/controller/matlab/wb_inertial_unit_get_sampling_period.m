function result = wb_inertial_unit_get_sampling_period(tag)
% Usage: wb_inertial_unit_get_sampling_period(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/inertialunit">here</a>

result = calllib('libController', 'wb_inertial_unit_get_sampling_period', tag);
