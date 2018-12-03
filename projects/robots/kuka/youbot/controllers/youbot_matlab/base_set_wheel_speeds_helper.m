function base_set_wheel_speeds_helper(speeds)
  for i = 1:4
    wb_motor_set_position(base_get_wheel(i), Inf)
    wb_motor_set_velocity(base_get_wheel(i), speeds(i))
  end
end
