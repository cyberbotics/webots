function result = wb_camera_recognition_save_segmentation_image(tag, filename, quality)
% Usage: wb_camera_recognition_save_segmentation_image(tag, filename, quality)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/camera">here</a>

result = calllib('libController', 'wb_camera_recognition_save_segmentation_image', tag, filename, quality);
