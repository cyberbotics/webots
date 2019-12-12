function result = wb_connector_get_presence(tag)
% Usage: wb_connector_get_presence(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/connector">here</a>

result = calllib('libController', 'wb_connector_get_presence', tag);
