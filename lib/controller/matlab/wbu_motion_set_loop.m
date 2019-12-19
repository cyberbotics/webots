function wbu_motion_set_loop(motionref, loop)
% Usage: wbu_motion_set_loop(motionref, loop)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motion">here</a>

calllib('libController', 'wbu_motion_set_loop', motionref, loop);
