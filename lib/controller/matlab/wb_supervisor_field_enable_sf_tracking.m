function result = wb_supervisor_field_enable_sf_tracking(field, sampling_period)
% Usage: wb_supervisor_field_enable_sf_tracking(field, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_field_enable_sf_tracking', field, sampling_period);
