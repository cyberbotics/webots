function result = wb_receiver_get_signal_strength(tag)
% Usage: wb_receiver_get_signal_strength(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/receiver">here</a>

result = calllib('libController', 'wb_receiver_get_signal_strength', tag);
