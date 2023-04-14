function epuck_avoid_collision

% time in [ms] of a simulation step
TIME_STEP = 64;

MAX_SPEED = 6.28;

% initialize devices
ps = [];
ps_names = [ "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7" ];

for i = 1:8
  ps(i) = wb_robot_get_device(convertStringsToChars(ps_names(i)));
  wb_distance_sensor_enable(ps(i), TIME_STEP);
end

left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0.0);
wb_motor_set_velocity(right_motor, 0.0);

% feedback loop: step simulation until receiving an exit event
while wb_robot_step(TIME_STEP) ~= -1
  % read sensors outputs
  ps_values = [];
  for i = 1:8
    ps_values(i) = wb_distance_sensor_get_value(ps(i));
  end

  % detect obstacles
  right_obstacle = ps_values(1) > 80.0 | ps_values(2) > 80.0 | ps_values(3) > 80.0;
  left_obstacle = ps_values(6) > 80.0 | ps_values(7) > 80.0 | ps_values(8) > 80.0;

  % initialize motor speeds at 50% of MAX_SPEED.
  left_speed  = 0.5 * MAX_SPEED;
  right_speed = 0.5 * MAX_SPEED;
  % modify speeds according to obstacles
  if left_obstacle
    % turn right
    left_speed   = left_speed + 0.5 * MAX_SPEED;
    right_speed  = right_speed - 0.5 * MAX_SPEED;
  elseif right_obstacle
    % turn left
    left_speed  = left_speed - 0.5 * MAX_SPEED;
    right_speed = right_speed + 0.5 * MAX_SPEED;
  end
  % write actuators inputs
  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;
end
