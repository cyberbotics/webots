function wb_connector_lock(tag)
% Usage: wb_connector_lock(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/connector">here</a>

calllib('libController', 'wb_connector_lock', tag);
