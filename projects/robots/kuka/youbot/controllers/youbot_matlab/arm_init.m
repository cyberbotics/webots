function arm_init
  arm_set_element(1,wb_robot_get_device('arm1'))
  arm_set_element(2,wb_robot_get_device('arm2'))
  arm_set_element(3,wb_robot_get_device('arm3'))
  arm_set_element(4,wb_robot_get_device('arm4'))
  arm_set_element(5,wb_robot_get_device('arm5'))
  wb_motor_set_velocity(arm_get_element(2), 0.5)
  arm_set_height('ARM_RESET')
  arm_set_orientation('ARM_FRONT')
end
