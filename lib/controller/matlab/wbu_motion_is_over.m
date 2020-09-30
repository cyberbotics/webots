function result = wbu_motion_is_over(motionref)
% Usage: wbu_motion_is_over(motionref)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motion">here</a>

result = calllib('libController', 'wbu_motion_is_over', motionref);
