function wbu_motion_delete(motionref)
% Usage: wbu_motion_delete(motionref)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/motion">here</a>

calllib('libController', 'wbu_motion_delete', motionref);
