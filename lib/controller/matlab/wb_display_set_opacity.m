function wb_display_set_opacity(tag, opacity)
% Usage: wb_display_set_opacity(tag, opacity)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

calllib('libController', 'wb_display_set_opacity', tag, opacity);
