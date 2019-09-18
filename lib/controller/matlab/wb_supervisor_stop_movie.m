function wb_supervisor_stop_movie()

% DEPRECATED use wb_supervisor_movie_stop_recording() instead
warning('wb_supervisor_stop_movie() is deprecated, use wb_supervisor_movie_stop_recording() instead');

% Usage: wb_supervisor_stop_movie()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_stop_movie');
