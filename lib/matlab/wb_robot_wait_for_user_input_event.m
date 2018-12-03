function result = wb_robot_wait_for_user_input_event(event_type, timeout)
% Usage: wb_robot_wait_for_user_input_event(event_type, timeout)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/robot">here</a>

result = calllib('libController', 'wb_robot_wait_for_user_input_event', event_type, timeout);
