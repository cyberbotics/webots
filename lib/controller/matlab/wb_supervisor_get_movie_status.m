function result = wb_supervisor_get_movie_status()

% DEPRECATED
warning('wb_supervisor_get_movie_status() is deprecated, use wb_supervisor_movie_is_ready() and wb_supervisor_movie_failed() instead');

% Usage: wb_supervisor_get_movie_status()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_get_movie_status');
