function wbu_motion_set_reverse(motionref, reverse)
% Usage: wbu_motion_set_reverse(motionref, reverse)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motion">here</a>

calllib('libController', 'wbu_motion_set_reverse', motionref, reverse);
