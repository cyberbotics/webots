function result = wbu_system_getenv(variable)
% Usage: wbu_system_getenv(variable)
% Matlab API for Webots

result = calllib('libController', 'wbu_system_getenv', variable);
