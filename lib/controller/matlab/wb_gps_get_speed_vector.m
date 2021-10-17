function result = wb_gps_get_speed_vector(tag)
% Usage: wb_gps_get_speed_vector(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/gps">here</a>

obj = calllib('libController', 'wb_gps_get_speed_vector', tag);
setdatatype(obj, 'doublePtr', 1, 3);
result = get(obj, 'Value')';
