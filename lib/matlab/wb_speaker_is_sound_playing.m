function result = wb_speaker_is_sound_playing(tag, sound)
% Usage: wb_speaker_is_sound_playing(tag, sound)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/speaker">here</a>

result = calllib('libController', 'wb_speaker_is_sound_playing', tag, sound);
