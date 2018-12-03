function wb_display_draw_line(tag, x1, y1, x2, y2)
% Usage: wb_display_draw_line(tag, x1, y1, x2, y2)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

calllib('libController', 'wb_display_draw_line', tag, x1, y1, x2, y2);
