function result = wb_receiver_get_emitter_direction(tag)
% Usage: wb_receiver_get_emitter_direction(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/receiver">here</a>

obj = calllib('libController', 'wb_receiver_get_emitter_direction', tag);
setdatatype(obj, 'doublePtr', 1, 3);
result = get(obj, 'Value')';
