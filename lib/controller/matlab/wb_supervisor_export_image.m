function wb_supervisor_export_image(filename, quality)
% Usage: wb_supervisor_export_image(filename, quality)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_export_image', filename, quality);
