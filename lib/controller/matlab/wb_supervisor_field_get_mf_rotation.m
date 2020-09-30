function rotation = wb_supervisor_field_get_mf_rotation(field, index)
% Usage: wb_supervisor_field_get_mf_rotation(field, index)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_field_get_mf_rotation', field, index);
setdatatype(obj,'doublePtr', 1, 4);
rotation = get(obj, 'Value');
