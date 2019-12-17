function points = wb_lidar_get_layer_point_cloud(tag,index)
% Usage: wb_lidar_get_layer_point_cloud(tag,index)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lidar">here</a>

size = calllib('libController', 'wb_lidar_get_horizontal_resolution', tag);
for n = 1:size
    obj = calllib('libController', 'wb_lidar_get_point', tag, n - 1 + index * size);
    points(n) = obj.Value;
end
