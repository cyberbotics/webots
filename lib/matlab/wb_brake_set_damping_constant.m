function wb_brake_set_damping_constant(tag, damping_constant)
% Usage: wb_brake_set_damping_constant(tag, damping_constant)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/brake">here</a>

calllib('libController', 'wb_brake_set_damping_constant', tag, damping_constant);
