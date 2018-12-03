function result = wbu_motion_get_duration(motionref)
% Usage: wbu_motion_get_duration(motionref)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motion">here</a>

result = calllib('libController', 'wbu_motion_get_duration', motionref);
