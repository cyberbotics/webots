% Description: MATLAB controller example for Webots

% uncomment the next two lines if you want to use
% MATLAB's desktop and interact with the controller
%desktop;
%keyboard;

TIME_STEP = wb_robot_get_basic_time_step();

right_shoulder_motor  = wb_robot_get_device('ShoulderR');
left_shoulder_motor   = wb_robot_get_device('ShoulderL');
right_upper_arm_motor = wb_robot_get_device('ArmUpperR');
left_upper_arm_motor  = wb_robot_get_device('ArmUpperL');
right_lower_arm_motor = wb_robot_get_device('ArmLowerR');
left_lower_arm_motor  = wb_robot_get_device('ArmLowerL');

left_shoulder_sensor  = wb_robot_get_device('ShoulderLS');
left_upper_arm_sensor = wb_robot_get_device('ArmUpperLS');
left_lower_arm_sensor = wb_robot_get_device('ArmLowerLS');

wb_motor_set_available_torque(left_shoulder_motor,0.0);
wb_motor_set_available_torque(left_upper_arm_motor,0.0);
wb_motor_set_available_torque(left_lower_arm_motor,0.0);

right_shoulder_min_position  = wb_motor_get_min_position(right_shoulder_motor);
right_shoulder_max_position  = wb_motor_get_max_position(right_shoulder_motor);
right_upper_arm_min_position = wb_motor_get_min_position(right_upper_arm_motor);
right_upper_arm_max_position = wb_motor_get_max_position(right_upper_arm_motor);
right_lower_arm_min_position = wb_motor_get_min_position(right_lower_arm_motor);
right_lower_arm_max_position = wb_motor_get_max_position(right_lower_arm_motor);

wb_position_sensor_enable(left_shoulder_sensor,TIME_STEP);
wb_position_sensor_enable(left_upper_arm_sensor,TIME_STEP);
wb_position_sensor_enable(left_lower_arm_sensor,TIME_STEP);

wb_console_print('-------Symmetry example of ROBOTIS OP2-------',WB_STDOUT);
wb_console_print('The right arm is free while the left one mimics it.',WB_STDOUT);
wb_console_print('In order to move the left arm, add a force to the right arm:',WB_STDOUT);
wb_console_print('keep alt pressed and select the right arm.',WB_STDOUT);
wb_console_print('Now you just have to move the mouse without releasing it.',WB_STDOUT);
wb_console_print('This example illustrates also the selfCollision which is activated by default',WB_STDOUT);
wb_console_print(sprintf('The basic time step is %g ms',TIME_STEP),WB_STDOUT);

while wb_robot_step(TIME_STEP) ~= -1

  right_shoulder_position  = -wb_position_sensor_get_value(left_shoulder_sensor);
  right_upper_arm_position = -wb_position_sensor_get_value(left_upper_arm_sensor);
  right_lower_arm_position = -wb_position_sensor_get_value(left_lower_arm_sensor);

  if (right_shoulder_position > right_shoulder_max_position)
   right_shoulder_position = right_shoulder_max_position;
  elseif (right_shoulder_position < right_shoulder_min_position)
   right_shoulder_position = right_shoulder_min_position;
  end

  if (right_upper_arm_position > right_upper_arm_max_position)
   right_upper_arm_position = right_upper_arm_max_position;
  elseif (right_upper_arm_position < right_upper_arm_min_position)
   right_upper_arm_position = right_upper_arm_min_position;
  end

  if (right_lower_arm_position > right_lower_arm_max_position)
   right_lower_arm_position = right_lower_arm_max_position;
  elseif (right_lower_arm_position < right_lower_arm_min_position)
   right_lower_arm_position = right_lower_arm_min_position;
  end

  wb_motor_set_position(right_shoulder_motor,right_shoulder_position);
  wb_motor_set_position(right_upper_arm_motor,right_upper_arm_position);
  wb_motor_set_position(right_lower_arm_motor,right_lower_arm_position);
end
