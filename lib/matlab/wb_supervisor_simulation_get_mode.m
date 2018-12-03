function result = wb_supervisor_simulation_get_mode()
% Usage: wb_supervisor_simulation_get_mode()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_simulation_get_mode');
