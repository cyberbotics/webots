% Matlab API for Webots
% Cleans up and unloads libController before quitting Matlab

if libisloaded('libController')
  calllib('libController','wb_robot_cleanup');
  unloadlibrary('libController');
end
