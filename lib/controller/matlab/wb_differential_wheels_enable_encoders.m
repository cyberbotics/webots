function wb_differential_wheels_enable_encoders(sampling_period)
% Usage: wb_differential_wheels_enable_encoders(sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/differentialwheels">here</a>

calllib('libController', 'wb_differential_wheels_enable_encoders', sampling_period);
