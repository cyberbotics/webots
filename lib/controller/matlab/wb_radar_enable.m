function wb_radar_enable(tag, sampling_period)
% Usage: wb_radar_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/radar">here</a>

calllib('libController', 'wb_radar_enable', tag, sampling_period);
