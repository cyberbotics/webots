function arm_perform_reset
  wb_motor_set_position(arm_get_element(1), 0.0)
  wb_motor_set_position(arm_get_element(2), 1.57)
  wb_motor_set_position(arm_get_element(3), -2.635)
  wb_motor_set_position(arm_get_element(4), 1.78)
  wb_motor_set_position(arm_get_element(5), 0.0)
end
