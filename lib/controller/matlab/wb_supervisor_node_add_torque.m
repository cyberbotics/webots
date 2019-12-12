function wb_supervisor_node_add_torque(noderef, torque, relative)
% Usage: wb_supervisor_node_add_torque(noderef, torque, relative)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_node_add_torque', noderef, torque, relative);
