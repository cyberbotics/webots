function contact_points = wb_supervisor_node_get_contact_points(noderef,include_descendants)
% Usage: wb_supervisor_node_get_contact_points(noderef,include_descendants)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

% need to stay in memory until wb_robot_step()
persistent pointer;

size = libpointer('int32');
contact_points_ref = calllib('libController','wb_supervisor_node_get_contact_points',noderef,include_descendants,size);
contact_points = []
for i = 1:size.Value
    contact_points(i) = contact_points_ref(i).Value;
end
