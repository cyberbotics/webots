function range = wb_range_finder_get_range_image(tag)
% Usage: wb_range_finder_get_range_image(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/rangefinder">here</a>

width = calllib('libController', 'wb_range_finder_get_width', tag);
height = calllib('libController', 'wb_range_finder_get_height', tag);
obj = calllib('libController', 'wb_range_finder_get_range_image', tag);
setdatatype(obj, 'singlePtr', width, height);
range = get(obj, 'Value')'; % fits the input format of the matlab imagesc() function
