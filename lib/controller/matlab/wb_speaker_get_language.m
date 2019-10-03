function result = wb_speaker_get_language(tag)
% Usage: wb_speaker_get_language(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/speaker">here</a>

result = calllib('libController', 'wb_speaker_get_language', tag);
