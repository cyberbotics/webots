function wb_display_fill_polygon(tag, x, y)
% Usage: wb_display_fill_polygon(tag, x, y)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

assert(~isvector(x), 'Invalid ''x'' argument: is not a vector');
assert(~isvector(y), 'Invalid ''y'' argument: is not a vector');
assert(numel(x) == numel(y), 'Invalid arguments: number of elements in ''x'' and ''y'' differ');
calllib('libController', 'wb_display_fill_polygon', tag, x, y, numel(x));
