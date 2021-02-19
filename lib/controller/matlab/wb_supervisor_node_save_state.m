function result = wb_supervisor_node_save_state(noderef, state_name)
% Usage: wb_supervisor_node_save_state(noderef, state_name)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_node_save_state', noderef, state_name);
