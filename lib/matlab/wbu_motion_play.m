function wbu_motion_play(motionref)
% Usage: wbu_motion_play(motionref)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motion">here</a>

calllib('libController', 'wbu_motion_play', motionref);
