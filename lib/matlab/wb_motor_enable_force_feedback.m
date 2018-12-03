function wb_motor_enable_force_feedback(tag, sampling_period)
% Usage: wb_motor_enable_force_feedback(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motor">here</a>

calllib('libController', 'wb_motor_enable_force_feedback', tag, sampling_period);
