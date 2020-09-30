function wb_supervisor_field_remove_sf(fieldref)
% Usage: wb_supervisor_field_remove_sf(fieldref)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

calllib('libController', 'wb_supervisor_field_remove_sf', fieldref);
