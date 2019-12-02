function wb_camera_recognition_enable(tag, sampling_period)
% Usage: wb_camera_recognition_enable(tag, sampling_period)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/camera">here</a>

calllib('libController', 'wb_camera_recognition_enable', tag, sampling_period);
