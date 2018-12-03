function wb_display_image_delete(tag, imageref)
% Usage: wb_display_image_delete(tag, imageref)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

calllib('libController', 'wb_display_image_delete', tag, imageref);
