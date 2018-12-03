function range = wb_lidar_get_range_image(tag)
% Usage: wb_lidar_get_range_image(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

width = calllib('libController', 'wb_lidar_get_horizontal_resolution', tag);
height = calllib('libController', 'wb_lidar_get_number_of_layers', tag);
obj = calllib('libController', 'wb_lidar_get_range_image', tag);
setdatatype(obj, 'singlePtr', width, height);
range = get(obj, 'Value')'; % fits the input format of the matlab imagesc() function
