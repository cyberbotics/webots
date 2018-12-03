function wb_display_draw_pixel(tag, x, y)
% Usage: wb_display_draw_pixel(tag, x, y)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

calllib('libController', 'wb_display_draw_pixel', tag, x, y);
