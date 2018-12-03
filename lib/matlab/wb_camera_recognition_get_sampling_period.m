function result = wb_camera_recognition_get_sampling_period(tag)
% Usage: wb_camera_recognition_get_sampling_period(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/camera">here</a>

result = calllib('libController', 'wb_camera_recognition_get_sampling_period', tag);
