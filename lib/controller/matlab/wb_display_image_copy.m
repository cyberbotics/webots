function result = wb_display_image_copy(tag, x, y, width, height)
% Usage: wb_display_image_copy(tag, x, y, width, height)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

result = calllib('libController', 'wb_display_image_copy', tag, x, y, width, height);
