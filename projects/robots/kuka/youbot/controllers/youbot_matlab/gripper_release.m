function gripper_release
  wb_motor_set_position(gripper_get_finger(1), 0.025)
  wb_motor_set_position(gripper_get_finger(2), 0.025)
end
