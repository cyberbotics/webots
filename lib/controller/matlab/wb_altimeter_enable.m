function wb_altimeter_enable(tag, sampling_period)
% Usage: wb_altimeter_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/altimeter">here</a>

calllib('libController', 'wb_altimeter_enable', tag, sampling_period);
