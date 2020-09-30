function wb_speaker_play_sound(left, right, sound, volume, pitch, balance, loop)
% Usage: wb_speaker_play_sound(left, right, sound, volume, pitch, balance, loop)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/speaker">here</a>

calllib('libController', 'wb_speaker_play_sound', left, right, sound, volume, pitch, balance, loop);
