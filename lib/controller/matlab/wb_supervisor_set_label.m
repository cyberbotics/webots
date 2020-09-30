function wb_supervisor_set_label(id, string, xpos, ypos, size, color, transparency, font)
% Usage: wb_supervisor_set_label(id, string, xpos, ypos, size, color, transparency, font)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

assert(numel(color) == 3, 'Invalid ''color'' argument: [red green blue] array expected');
colorbytes = 65536 * 255 * color(1) + 256 * 255 * color(2) + 255 * color(3);
obj = calllib('libController', 'wb_supervisor_set_label', id, string, xpos, ypos, size, colorbytes, transparency, font);
