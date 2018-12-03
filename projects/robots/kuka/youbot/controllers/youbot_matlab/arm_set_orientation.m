function arm_set_orientation(orientation)
  switch orientation
    case 'ARM_BACK_LEFT'
      wb_motor_set_position(arm_get_element(1), -2.949)
    case 'ARM_LEFT'
      wb_motor_set_position(arm_get_element(1), -pi/2)
    case 'ARM_FRONT_LEFT'
      wb_motor_set_position(arm_get_element(1), -0.2)
    case 'ARM_FRONT'
      wb_motor_set_position(arm_get_element(1), 0.0)
    case 'ARM_FRONT_RIGHT'
      wb_motor_set_position(arm_get_element(1), 0.2)
    case 'ARM_RIGHT'
      wb_motor_set_position(arm_get_element(1), pi/2)
    case 'ARM_BACK_RIGHT'
      wb_motor_set_position(arm_get_element(1), 2.949)
    otherwise
      wb_console_print('arm_set_side() called with a wrong argument',WB_STDERR)
      return;
  end
  arm_set_current_orientation(orientation)
end
