function rgb = wb_camera_get_image(tag)
% Usage: wb_camera_get_image(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/camera">here</a>

width = calllib('libController', 'wb_camera_get_width', tag);
height = calllib('libController', 'wb_camera_get_height', tag);
pointer = calllib('libController', 'wb_camera_get_image',tag);
setdatatype(pointer, 'uint8Ptr', 4 * width * height, 1);
rgba = permute(reshape(get(pointer, 'Value'), 4, width, height), [3 2 1]);
rgb(:,:,[1 2 3]) = rgba(:,:,[3 2 1]);
