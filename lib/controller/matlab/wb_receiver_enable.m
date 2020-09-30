function wb_receiver_enable(tag, sampling_period)
% Usage: wb_receiver_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/receiver">here</a>

calllib('libController', 'wb_receiver_enable', tag, sampling_period);
