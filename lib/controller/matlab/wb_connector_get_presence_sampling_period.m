function result = wb_connector_get_presence_sampling_period(tag)
% Usage: wb_connector_get_presence_sampling_period(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/connector">here</a>

result = calllib('libController', 'wb_connector_get_presence_sampling_period', tag);
