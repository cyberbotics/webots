function wb_motor_disable_torque_feedback(tag)
% Usage: wb_motor_disable_torque_feedback(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motor">here</a>

calllib('libController', 'wb_motor_disable_torque_feedback', tag);
