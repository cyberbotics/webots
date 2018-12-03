function wb_camera_recognition_disable(tag)
% Usage: wb_camera_recognition_disable(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/camera">here</a>

calllib('libController', 'wb_camera_recognition_disable', tag);
