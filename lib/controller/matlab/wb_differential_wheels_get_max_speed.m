function result = wb_differential_wheels_get_max_speed()
% Usage: wb_differential_wheels_get_max_speed()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/differentialwheels">here</a>

result = calllib('libController', 'wb_differential_wheels_get_max_speed');
