function wb_display_set_font(tag, font, size, anti_aliasing)
% Usage: wb_display_set_font(tag, font, size, anti_aliasing)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

calllib('libController', 'wb_display_set_font', tag, font, size, anti_aliasing);
