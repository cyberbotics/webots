function wb_receiver_disable(tag)
% Usage: wb_receiver_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/receiver">here</a>

calllib('libController', 'wb_receiver_disable', tag);
