function result = wb_emitter_send(tag, data)
% Usage: wb_emitter_send(tag, data)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/emitter">here</a>

w=whos('data');
result = calllib('libController', 'wb_emitter_send', tag, data, w.bytes);
