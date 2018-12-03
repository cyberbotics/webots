function wb_pen_set_ink_color(tag, color, density)
% Usage: wb_pen_set_ink_color(tag, color, density)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/pen">here</a>

assert(numel(color) == 3, 'Invalid ''color'' argument: [red green blue] array expected');
colorbytes = 65536 * 255 * color(1) + 256 * 255 * color(2) + 255 * color(3);
calllib('libController', 'wb_pen_set_ink_color', tag, colorbytes, density);
