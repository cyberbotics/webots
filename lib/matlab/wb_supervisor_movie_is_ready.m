function result = wb_supervisor_movie_is_ready()
% Usage: wb_supervisor_movie_is_ready()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_movie_is_ready');
