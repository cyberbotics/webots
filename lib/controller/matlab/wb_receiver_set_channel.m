function wb_receiver_set_channel(tag, channel)
% Usage: wb_receiver_set_channel(tag, channel)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/receiver">here</a>

calllib('libController', 'wb_receiver_set_channel', tag, channel);
