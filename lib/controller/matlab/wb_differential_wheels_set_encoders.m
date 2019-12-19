function wb_differential_wheels_set_encoders(left, right)
% Usage: wb_differential_wheels_set_encoders(left, right)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/differentialwheels">here</a>

calllib('libController', 'wb_differential_wheels_set_encoders', left, right);
