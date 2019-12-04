function result = wb_emitter_get_range(tag)
% Usage: wb_emitter_get_range(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/emitter">here</a>

result = calllib('libController', 'wb_emitter_get_range', tag);
