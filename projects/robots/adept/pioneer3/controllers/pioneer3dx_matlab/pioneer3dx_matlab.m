% Description: MATLAB controller example for Webots
function pioneer3dx_matlab

% uncomment the next two lines if you want to use
% MATLAB's desktop and interact with the controller
%desktop;
%keyboard;

time_step  = wb_robot_get_basic_time_step();
left_wheel  = wb_robot_get_device('left wheel');
right_wheel = wb_robot_get_device('right wheel');
red_led(1) = wb_robot_get_device('red led 1');
red_led(2) = wb_robot_get_device('red led 2');
red_led(3) = wb_robot_get_device('red led 3');
MAX_SENSOR_NUMBER = 16;
for i = [1:MAX_SENSOR_NUMBER]
  sonar(i) = wb_robot_get_device(strcat('so', num2str(i - 1)));
  wb_distance_sensor_enable(sonar(i), time_step);
end
wb_motor_set_position(left_wheel, Inf);
wb_motor_set_position(right_wheel, Inf);

while wb_robot_step(time_step) ~= -1
  for i = [1:MAX_SENSOR_NUMBER]
    value = wb_distance_sensor_get_value(sonar(i));

    if (value == 0)
      speed_modifier(i) = 0;
    else
      distance = 5.0 * (1 - (value / 1024)); % lookup table inverse.
      if (distance < 0.5)
        speed_modifier(i) = (0.5 - distance) * 3000;
      else
        speed_modifier(i) = 0;
      end
    end

  end

  left_speed = 5;
  right_speed = 5;
  right = speed_modifier(1) + speed_modifier(2) + speed_modifier(3) + speed_modifier(4);
  left = speed_modifier(5) + speed_modifier(6) + speed_modifier(7) + speed_modifier(8);
  if (right > 1000)
    right_speed = -3;
    left_speed= 3;
  end
  if (left > 1000 & left > right)
    left_speed = -3;
    right_speed = 3;
  end
  wb_motor_set_velocity(left_wheel, left_speed);
  wb_motor_set_velocity(right_wheel, right_speed);
end
