function depth = wb_range_finder_image_get_depth(image,width,x,y)
% Usage: wb_range_finder_image_get_depth(image,width,x,y)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/rangefinder">here</a>

depth = image(x,y);
