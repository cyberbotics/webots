function wb_motor_set_available_torque(tag, torque)
% Usage: wb_motor_set_available_torque(tag, torque)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motor">here</a>

calllib('libController', 'wb_motor_set_available_torque', tag, torque);
