function targets = wb_radar_get_targets(tag)
% Usage: wb_radar_get_targets(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/radar">here</a>

rate = calllib('libController', 'wb_radar_get_sampling_period', tag);
if rate <= 0
    targets = []
    calllib('libController', 'wb_console_print', 'Warning: wb_radar_get_targets() called for a disabled radar! Please call wb_radar_enable() before.', 2);
else
    size = calllib('libController', 'wb_radar_get_number_of_targets', tag);
    for n = 1:size
        obj = calllib('libController', 'wb_radar_get_target', tag, n - 1);
        targets(n) = obj.Value;
    end
end
