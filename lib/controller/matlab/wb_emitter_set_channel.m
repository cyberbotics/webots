function wb_emitter_set_channel(tag, channel)
% Usage: wb_emitter_set_channel(tag, channel)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/emitter">here</a>

calllib('libController', 'wb_emitter_set_channel', tag, channel);
