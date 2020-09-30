function result = wb_supervisor_save_world(filename)
% Usage: wb_supervisor_save_world(filename)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

if nargin == 0
  nullpointer = libpointer;
  result = calllib('libController', 'wb_supervisor_save_world', nullpointer);
else
  result = calllib('libController', 'wb_supervisor_save_world', filename);
end
