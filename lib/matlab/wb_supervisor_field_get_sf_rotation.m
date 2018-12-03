function rotation = wb_supervisor_field_get_sf_rotation(fieldref)
% Usage: wb_supervisor_field_get_sf_rotation(fieldref)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_field_get_sf_rotation', fieldref);
setdatatype(obj,'doublePtr', 1, 4);
rotation = get(obj, 'Value');
