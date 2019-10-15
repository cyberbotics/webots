function result = wb_receiver_get_queue_length(tag)
% Usage: wb_receiver_get_queue_length(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/receiver">here</a>

result = calllib('libController', 'wb_receiver_get_queue_length', tag);
