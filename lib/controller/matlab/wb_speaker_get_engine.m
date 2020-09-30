function result = wb_speaker_get_engine(tag)
% Usage: wb_speaker_get_engine(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/speaker">here</a>

result = calllib('libController', 'wb_speaker_get_engine', tag);
