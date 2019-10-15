function wb_display_draw_text(tag, txt, x, y)
% Usage: wb_display_draw_text(tag, txt, x, y)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

calllib('libController', 'wb_display_draw_text', tag, txt, x, y);
