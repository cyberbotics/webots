/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   A simple biped Robot.
                  This controller turns off the robot motors after 12.8 seconds.
 */

#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 64

int main() {
  // initialize Webots
  wb_robot_init();

  // get joints
  WbDeviceTag joint[2][3], head;
  joint[0][0] = wb_robot_get_device("left hip");
  joint[0][1] = wb_robot_get_device("left knee");
  joint[0][2] = wb_robot_get_device("left ankle");
  joint[1][0] = wb_robot_get_device("right hip");
  joint[1][1] = wb_robot_get_device("right knee");
  joint[1][2] = wb_robot_get_device("right ankle");
  head = wb_robot_get_device("head");

  wb_motor_set_control_pid(joint[0][0], 5.0, 0.0, 0.0);
  wb_motor_set_velocity(joint[0][0], 0.5);
  wb_motor_set_velocity(joint[0][2], 0.5);
  wb_motor_set_velocity(joint[1][0], 0.5);
  wb_motor_set_velocity(joint[1][2], 0.5);
  wb_motor_set_position(joint[0][0], 0.3);
  wb_motor_set_position(joint[0][1], -0.6);
  wb_motor_set_position(joint[0][2], 0.3);
  wb_motor_set_position(joint[1][0], 0.3);
  wb_motor_set_position(joint[1][1], -0.6);
  wb_motor_set_position(joint[1][2], 0.3);

  // hold position for 12.8 seconds
  int i;
  for (i = 0; i < 200; i++)
    wb_robot_step(TIME_STEP);

  // turn off the motors and let the robot collapse after 12.8 seconds
  wb_motor_set_available_torque(joint[0][0], 0.0);
  wb_motor_set_available_torque(joint[0][1], 0.0);
  wb_motor_set_available_torque(joint[0][2], 0.0);
  wb_motor_set_available_torque(joint[1][0], 0.0);
  wb_motor_set_available_torque(joint[1][1], 0.0);
  wb_motor_set_available_torque(joint[1][2], 0.0);

  // rotate head
  wb_motor_set_position(head, -INFINITY);

  // forever
  for (;;)
    wb_robot_step(TIME_STEP);

  return 0;  // never reached
}
