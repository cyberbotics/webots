function wb_display_image_save(tag, imageref, filename)
% Usage: wb_display_image_save(tag, imageref, filename)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

calllib('libController', 'wb_display_image_save', tag, imageref, filename);
