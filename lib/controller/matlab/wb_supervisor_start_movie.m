function wb_supervisor_start_movie(filename, width, height, codec, quality, acceleration, caption)

% DEPRECATED use wb_supervisor_movie_start_recording() instead
warning('wb_supervisor_start_movie() is deprecated, use wb_supervisor_movie_start_recording() instead');

% Usage: wb_supervisor_start_movie(filename, width, height, codec, quality, acceleration, caption)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_start_movie', filename, width, height, codec, quality, acceleration, caption);
