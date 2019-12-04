function result = wbu_motion_set_time(motionref, time)
% Usage: wbu_motion_set_time(motionref, time)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motion">here</a>

result = calllib('libController', 'wbu_motion_set_time', motionref, time);
