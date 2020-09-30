function wb_pen_write(tag, write)
% Usage: wb_pen_write(tag, write)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/pen">here</a>

calllib('libController', 'wb_pen_write', tag, write);
