function wb_gps_enable(tag, sampling_period)
% Usage: wb_gps_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/gps">here</a>

calllib('libController', 'wb_gps_enable', tag, sampling_period);
