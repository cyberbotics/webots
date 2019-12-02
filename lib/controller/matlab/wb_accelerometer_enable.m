function wb_accelerometer_enable(tag, sampling_period)
% Usage: wb_accelerometer_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/accelerometer">here</a>

calllib('libController', 'wb_accelerometer_enable', tag, sampling_period);
