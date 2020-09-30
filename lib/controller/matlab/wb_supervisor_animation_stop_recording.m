function result = wb_supervisor_animation_stop_recording()
% Usage: wb_supervisor_animation_stop_recording()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_animation_stop_recording');
