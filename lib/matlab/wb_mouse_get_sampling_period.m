function result = wb_mouse_get_sampling_period()
% Usage: wb_mouse_get_sampling_period()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/mouse">here</a>

result = calllib('libController', 'wb_mouse_get_sampling_period');
