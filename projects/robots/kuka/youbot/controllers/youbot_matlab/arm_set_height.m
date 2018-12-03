function arm_set_height(height)
  switch height
    case 'ARM_FRONT_FLOOR'
      wb_motor_set_position(arm_get_element(2), -0.97)
      wb_motor_set_position(arm_get_element(3), -1.55)
      wb_motor_set_position(arm_get_element(4), -0.61)
      wb_motor_set_position(arm_get_element(5),  0.0)
    case 'ARM_FRONT_PLATE'
      wb_motor_set_position(arm_get_element(2), -0.62)
      wb_motor_set_position(arm_get_element(3), -0.98)
      wb_motor_set_position(arm_get_element(4), -1.53)
      wb_motor_set_position(arm_get_element(5), 0.0)
    case 'ARM_FRONT_CARDBOARD_BOX'
      wb_motor_set_position(arm_get_element(2),  0.0)
      wb_motor_set_position(arm_get_element(3), -0.77)
      wb_motor_set_position(arm_get_element(4), -1.21)
      wb_motor_set_position(arm_get_element(5),  0.0)
    case 'ARM_RESET'
      wb_motor_set_position(arm_get_element(2),  1.57)
      wb_motor_set_position(arm_get_element(3), -2.635)
      wb_motor_set_position(arm_get_element(4),  1.78)
      wb_motor_set_position(arm_get_element(5),  0.0)
    case 'ARM_BACK_PLATE_HIGH'
      wb_motor_set_position(arm_get_element(2),  0.678)
      wb_motor_set_position(arm_get_element(3),  0.682)
      wb_motor_set_position(arm_get_element(4),  1.74)
      wb_motor_set_position(arm_get_element(5),  0.0)
    case 'ARM_BACK_PLATE_LOW'
      wb_motor_set_position(arm_get_element(2), 0.92)
      wb_motor_set_position(arm_get_element(3), 0.42)
      wb_motor_set_position(arm_get_element(4), 1.78)
      wb_motor_set_position(arm_get_element(5),  0.0)
    case 'ARM_HANOI_PREPARE'
      wb_motor_set_position(arm_get_element(2), -0.4)
      wb_motor_set_position(arm_get_element(3), -1.2)
      wb_motor_set_position(arm_get_element(4), -pi/2)
      wb_motor_set_position(arm_get_element(5),  pi/2)
    otherwise
      wb_console_print('arm_height() called with a wrong argument',WB_STDERR)
      return;
  end
  arm_set_current_height(height)
end
