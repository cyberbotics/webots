function gripper_init
  gripper_set_finger(1, wb_robot_get_device('finger::left'))
  gripper_set_finger(2, wb_robot_get_device('finger::right'))
  wb_motor_set_velocity(gripper_get_finger(1), 0.03)
  wb_motor_set_velocity(gripper_get_finger(2), 0.03)
end
