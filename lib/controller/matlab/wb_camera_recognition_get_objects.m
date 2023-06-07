function objects = wb_camera_recognition_get_objects(tag)
% Usage: wb_camera_recognition_get_objects(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/camera">here</a>

rate = calllib('libController', 'wb_camera_recognition_get_sampling_period', tag);
if rate <= 0
    objects = []
    calllib('libController', 'wb_console_print', 'Warning: wb_camera_recognition_get_objects() called for a disabled device! Please use: wb_camera_recognition_enable() before.', 2);
else
    size = calllib('libController', 'wb_camera_recognition_get_number_of_objects', tag);
    for n = 1:size
        obj = calllib('libController', 'wb_camera_recognition_get_object', tag, n - 1);
        objects(n) = obj.Value;
    end
end
