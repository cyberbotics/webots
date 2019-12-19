function wb_speaker_speak(tag, text, volume)
% Usage: wb_speaker_speak(tag, text, volume)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/speaker">here</a>

calllib('libController', 'wb_speaker_speak', tag, text, volume);
