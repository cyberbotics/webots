function wb_supervisor_simulation_quit(status)
% Usage: wb_supervisor_simulation_quit(status)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_simulation_quit', status);
