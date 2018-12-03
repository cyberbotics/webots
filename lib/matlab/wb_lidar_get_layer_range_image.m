function range = wb_lidar_get_layer_range_image(tag,layer)
% Usage: wb_lidar_get_layer_range_image(tag,layer)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

width = calllib('libController', 'wb_lidar_get_horizontal_resolution', tag);
obj = calllib('libController', 'wb_lidar_get_layer_range_image', tag, layer);
setdatatype(obj, 'singlePtr', width, 1);
range = get(obj, 'Value')'; % fits the input format of the matlab imagesc() function
