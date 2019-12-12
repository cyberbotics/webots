function result = wb_mouse_get_state()
% Usage: wb_mouse_get_state()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/mouse">here</a>

obj = calllib('libController', 'wb_mouse_get_state_pointer');
result = obj.Value;
