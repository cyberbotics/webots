function wb_camera_set_exposure(tag, exposure)
% Usage: wb_camera_set_exposure(tag, exposure)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/camera">here</a>

calllib('libController', 'wb_camera_set_exposure', tag, exposure);
