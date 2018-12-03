function result = wb_emitter_get_buffer_size(tag)
% Usage: wb_emitter_get_buffer_size(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/emitter">here</a>

result = calllib('libController', 'wb_emitter_get_buffer_size', tag);
