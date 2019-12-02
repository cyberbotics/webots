function wb_motor_set_acceleration(tag, acceleration)
% Usage: wb_motor_set_acceleration(tag, acceleration)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motor">here</a>

calllib('libController', 'wb_motor_set_acceleration', tag, acceleration);
