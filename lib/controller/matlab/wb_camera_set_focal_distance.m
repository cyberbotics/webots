function wb_camera_set_focal_distance(tag, focal_distance)
% Usage: wb_camera_set_focal_distance(tag, focal_distance)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/camera">here</a>

calllib('libController', 'wb_camera_set_focal_distance', tag, focal_distance);
