function wb_camera_recognition_set_segmentation(tag, value)
% Usage: wb_camera_recognition_set_segmentation(tag, value)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/camera">here</a>

calllib('libController', 'wb_camera_recognition_set_segmentation', tag, value);
