function gripper_grip
  wb_motor_set_position(gripper_get_finger(1), 0.0)
  wb_motor_set_position(gripper_get_finger(2), 0.0)
end
