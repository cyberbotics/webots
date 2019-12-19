function result = wb_mouse_is_3d_position_enabled()
% Usage: wb_mouse_is_3d_position_enabled()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/mouse">here</a>

result = calllib('libController', 'wb_mouse_is_3d_position_enabled');
