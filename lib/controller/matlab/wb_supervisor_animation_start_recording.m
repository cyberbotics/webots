function result = wb_supervisor_animation_start_recording(filename)
% Usage: wb_supervisor_animation_start_recording(filename)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_animation_start_recording', filename);
