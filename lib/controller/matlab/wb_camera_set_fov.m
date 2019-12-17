function wb_camera_set_fov(tag, fov)
% Usage: wb_camera_set_fov(tag, fov)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/camera">here</a>

calllib('libController', 'wb_camera_set_fov', tag, fov);
