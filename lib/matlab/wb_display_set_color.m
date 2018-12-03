function wb_display_set_color(tag, color)
% Usage: wb_display_set_color(tag, color)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

assert(numel(color) == 3, 'Invalid ''color'' argument: [red green blue] array expected');
colorbytes = 65536 * 255 * color(1) + 256 * 255 * color(2) + 255 * color(3);
calllib('libController', 'wb_display_set_color', tag, colorbytes);
