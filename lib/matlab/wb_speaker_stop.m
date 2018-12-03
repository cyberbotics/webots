function wb_speaker_stop(tag, sound)
% Usage: wb_speaker_stop(tag, sound)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/speaker">here</a>

calllib('libController', 'wb_speaker_stop', tag, sound);
