function wb_gyro_disable(tag)
% Usage: wb_gyro_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/gyro">here</a>

calllib('libController', 'wb_gyro_disable', tag);
