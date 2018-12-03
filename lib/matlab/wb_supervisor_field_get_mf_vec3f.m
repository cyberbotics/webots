function vec3f = wb_supervisor_field_get_mf_vec3f(field, index)
% Usage: wb_supervisor_field_get_mf_vec3f(field, index)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_field_get_mf_vec3f', field, index);
setdatatype(obj,'doublePtr', 1, 3);
vec3f = get(obj, 'Value');
