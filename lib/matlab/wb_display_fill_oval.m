function wb_display_fill_oval(tag, cx, cy, a, b)
% Usage: wb_display_fill_oval(tag, cx, cy, a, b)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

calllib('libController', 'wb_display_fill_oval', tag, cx, cy, a, b);
