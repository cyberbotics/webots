function result = wb_skin_get_bone_count(tag)
% Usage: wb_skin_get_bone_count(tag)
% Matlab API for Webots

result = calllib('libController', 'wb_skin_get_bone_count', tag);
