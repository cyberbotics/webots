function contact_points = wb_supervisor_node_get_contact_points(noderef,include_descendants)
% Usage: wb_supervisor_node_get_contact_points(noderef,include_descendants)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

% need to stay in memory until wb_robot_step()
persistent pointer;

size = libpointer('int32Ptr', 0);
contact_points_ref = calllib('libController','wb_supervisor_node_get_contact_points',noderef,include_descendants,size);
setdatatype(contact_points_ref, 'WbContactPointPtr');
if size.value <= 0
    contact_points = [];
else
    for i = 1:size.value
        contact_points_ref_array = contact_points_ref + (i - 1);
        contact_points(i) = contact_points_ref_array.value;
    end
end
