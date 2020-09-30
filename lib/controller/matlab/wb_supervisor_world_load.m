function wb_supervisor_world_load(filename)
% Usage: wb_supervisor_world_load(filename)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_world_load', filename);
