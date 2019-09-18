function wb_connector_enable_presence(tag, sampling_period)
% Usage: wb_connector_enable_presence(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/connector">here</a>

calllib('libController', 'wb_connector_enable_presence', tag, sampling_period);
