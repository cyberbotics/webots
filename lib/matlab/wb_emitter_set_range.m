function wb_emitter_set_range(tag, range)
% Usage: wb_emitter_set_range(tag, range)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/emitter">here</a>

calllib('libController', 'wb_emitter_set_range', tag, range);
