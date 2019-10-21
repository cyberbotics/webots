function wb_supervisor_movie_start_recording(filename, width, height, codec, quality, acceleration, caption)
% Usage: wb_supervisor_movie_start_recording(filename, width, height, codec, quality, acceleration, caption)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_movie_start_recording', filename, width, height, codec, quality, acceleration, caption);
