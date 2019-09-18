function wb_supervisor_simulation_set_mode(mode)
% Usage: wb_supervisor_simulation_set_mode(mode)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_simulation_set_mode', mode);
