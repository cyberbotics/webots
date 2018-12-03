function result = wb_skin_get_bone_orientation(tag, index, absolute)
% Usage: wb_skin_get_bone_orientation(tag, index, absolute)
% Matlab API for Webots

obj = calllib('libController', 'wb_skin_get_bone_orientation', tag, index, absolute);
setdatatype(obj,'doublePtr', 1, 4);
result = get(obj, 'Value');
