function result = wb_display_image_load(tag, filename)
% Usage: wb_display_image_load(tag, filename)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

result = calllib('libController', 'wb_display_image_load', tag, filename);
