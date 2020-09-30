function vec3f = wb_supervisor_field_get_sf_vec3f(fieldref)
% Usage: wb_supervisor_field_get_sf_vec3f(fieldref)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_field_get_sf_vec3f', fieldref);
setdatatype(obj,'doublePtr', 1, 3);
vec3f = get(obj, 'Value');
