function wb_motor_set_control_pid(tag, p, i, d)
% Usage: wb_motor_set_control_pid(tag, p, i, d)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motor">here</a>

calllib('libController', 'wb_motor_set_control_pid', tag, p, i, d);
