function result = wb_gps_convert_to_degrees_minutes_seconds(decimal_degrees)
% Usage: wb_gps_convert_to_degrees_minutes_seconds(decimal_degrees)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/gps">here</a>

result = calllib('libController', 'wb_gps_convert_to_degrees_minutes_seconds', decimal_degrees);
