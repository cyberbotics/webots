function result = wb_supervisor_node_disable_contact_points_tracking(noderef)
% Usage: wb_supervisor_node_disable_contact_points_tracking(noderef)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_node_disable_contact_points_tracking', noderef);
