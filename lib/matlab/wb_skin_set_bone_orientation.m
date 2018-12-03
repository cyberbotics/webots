function wb_skin_set_bone_orientation(tag, index, values, absolute)
% Usage: wb_skin_set_bone_orientation(tag, index, values, absolute)
% Matlab API for Webots

assert(numel(values) == 4, 'Invalid ''values'' argument: 1x4 or 4x1 array expected');
calllib('libController', 'wb_skin_set_bone_orientation', tag, index, values, absolute);
