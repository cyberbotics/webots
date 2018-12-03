function wb_display_image_paste(tag, imageref, x, y, blend)
% Usage: wb_display_image_paste(tag, imageref, x, y, blend)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

calllib('libController', 'wb_display_image_paste', tag, imageref, x, y, blend);
