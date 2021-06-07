function result = wb_supervisor_field_disable_sf_tracking(field)
% Usage: wb_supervisor_field_disable_sf_tracking(field)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_field_disable_sf_tracking', field);
