function four_wheels_collision_avoidance

TIME_STEP = 64;
ds = [];
ds_names = [ "ds_right", "ds_left" ];
for i = 1:2
  ds(i) = wb_robot_get_device(convertStringsToChars(ds_names(i)));
  wb_distance_sensor_enable(ds(i), TIME_STEP);
end
wheels = [];
wheels_names = [ "wheel1", "wheel2", "wheel3", "wheel4" ];
for i = 1:4
  wheels(i) = wb_robot_get_device(convertStringsToChars(wheels_names(i)));
  wb_motor_set_position(wheels(i), inf);
  wb_motor_set_velocity(wheels(i), 0.0);
end
avoid_obstacle_counter = 0;

while wb_robot_step(TIME_STEP) ~= -1
  left_speed = 1.0;
  right_speed = 1.0;
  if avoid_obstacle_counter > 0
    avoid_obstacle_counter = avoid_obstacle_counter - 1;
    left_speed = 1.0;
    right_speed = -1.0;
  else % read sensors
    for i = 1:2
      if wb_distance_sensor_get_value(ds(i)) < 950.0
        avoid_obstacle_counter = 100;
      end
    end
  end
  wb_motor_set_velocity(wheels(1), left_speed);
  wb_motor_set_velocity(wheels(2), right_speed);
  wb_motor_set_velocity(wheels(3), left_speed);
  wb_motor_set_velocity(wheels(4), right_speed);
  % if your code plots some graphics, it needs to flushed like this:
  drawnow;
end
