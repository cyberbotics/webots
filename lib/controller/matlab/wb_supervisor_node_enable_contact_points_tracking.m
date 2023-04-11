function result = wb_supervisor_node_enable_contact_points_tracking(noderef, sampling_period, include_descendants)
% Usage: wb_supervisor_node_enable_contact_points_tracking(noderef, sampling_period, include_descendants)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

result = calllib('libController', 'wb_supervisor_node_enable_contact_points_tracking', noderef, sampling_period, include_descendants);
