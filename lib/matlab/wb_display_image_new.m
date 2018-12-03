function imageref = wb_display_image_new(tag,rgb,format)
% Usage: wb_camera_get_image(tag,rgb,format)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/display">here</a>

% need to stay in memory until wb_robot_step()
persistent pointer;

height=size(rgb,1);
width=size(rgb,2);
pointer = libpointer('uint8Ptr',permute(rgb,[3 2 1]));
imageref = calllib('libController','wb_display_image_new',tag,width,height,pointer,format);
