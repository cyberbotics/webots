function vec2f = wb_supervisor_field_get_sf_vec2f(fieldref)
% Usage: wb_supervisor_field_get_sf_vec2f(fieldref)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_field_get_sf_vec2f', fieldref);
setdatatype(obj,'doublePtr', 1, 2);
vec2f = get(obj, 'Value');
