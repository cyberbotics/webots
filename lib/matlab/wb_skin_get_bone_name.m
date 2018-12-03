function result = wb_skin_get_bone_name(tag, index)
% Usage: wb_skin_get_bone_name(tag, index)
% Matlab API for Webots

result = calllib('libController', 'wb_skin_get_bone_name', tag, index);
