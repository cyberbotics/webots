function wb_motor_set_velocity(tag, velocity)
% Usage: wb_motor_set_velocity(tag, velocity)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motor">here</a>

calllib('libController', 'wb_motor_set_velocity', tag, velocity);
