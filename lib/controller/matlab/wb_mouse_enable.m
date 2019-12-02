function wb_mouse_enable(sampling_period)
% Usage: wb_mouse_enable(sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/mouse">here</a>

calllib('libController', 'wb_mouse_enable', sampling_period);
