function wb_supervisor_simulation_physics_reset()
% DEPRECATED use wb_supervisor_simulation_reset_physics() instead
warning('wb_supervisor_simulation_physics_reset() is deprecated, use wb_supervisor_simulation_reset_physics() instead');

% Usage: wb_supervisor_simulation_physics_reset()
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_simulation_reset_physics');
