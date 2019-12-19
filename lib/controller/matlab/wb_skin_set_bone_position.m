function wb_skin_set_bone_position(tag, index, values, absolute)
% Usage: wb_skin_set_bone_position(tag, index, values, absolute)
% Matlab API for Webots

assert(numel(values) == 3, 'Invalid ''values'' argument: 1x3 or 3x1 array expected');
calllib('libController', 'wb_skin_set_bone_position', tag, index, values, absolute);
