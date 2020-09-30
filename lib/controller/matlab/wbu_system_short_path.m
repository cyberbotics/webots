function result = wbu_system_short_path(path)
% Usage: wbu_system_short_path(path)
% Matlab API for Webots

result = calllib('libController', 'wbu_system_short_path', path);
