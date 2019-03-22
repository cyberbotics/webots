/*
 * Copyright 1996-2019 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/camera.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <stdio.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag front_motor = wb_robot_get_device("front motor");
  WbDeviceTag rear_motor = wb_robot_get_device("rear motor");

  WbDeviceTag front_left_track = wb_robot_get_device("left front track");
  WbDeviceTag front_right_track = wb_robot_get_device("right front track");
  WbDeviceTag rear_left_track = wb_robot_get_device("left rear track");
  WbDeviceTag rear_right_track = wb_robot_get_device("right rear track");

  WbDeviceTag arm_motors[7];
  char buffer[32];
  int i;
  for (i = 0; i < 7; ++i) {
    sprintf(buffer, "arm %d motor", i);
    arm_motors[i] = wb_robot_get_device(buffer);
  }

  wb_motor_set_position(front_left_track, INFINITY);
  wb_motor_set_position(front_right_track, INFINITY);
  wb_motor_set_position(rear_left_track, INFINITY);
  wb_motor_set_position(rear_right_track, INFINITY);
  wb_motor_set_velocity(front_left_track, -0.1);
  wb_motor_set_velocity(front_right_track, -0.1);
  wb_motor_set_velocity(rear_left_track, 0.1);
  wb_motor_set_velocity(rear_right_track, 0.1);

  wb_robot_step(20000);

  wb_motor_set_velocity(front_left_track, 0.0);
  wb_motor_set_velocity(front_right_track, 0.0);
  wb_motor_set_velocity(rear_left_track, 0.0);
  wb_motor_set_velocity(rear_right_track, 0.0);
  wb_motor_set_position(front_motor, 1.0);
  wb_motor_set_position(rear_motor, -0.2);

  wb_robot_step(7000);

  wb_motor_set_velocity(front_left_track, -0.1);
  wb_motor_set_velocity(front_right_track, -0.1);
  wb_motor_set_velocity(rear_left_track, 0.1);
  wb_motor_set_velocity(rear_right_track, 0.1);

  wb_robot_step(13000);

  wb_motor_set_position(front_motor, 0.25);

  wb_robot_step(60000);

  wb_motor_set_position(front_motor, 0.0);
  wb_motor_set_position(rear_motor, 0.0);

  wb_robot_step(5000);

  wb_motor_set_velocity(front_left_track, 0.1);
  wb_motor_set_velocity(front_right_track, 0.1);
  wb_motor_set_velocity(rear_left_track, 0.1);
  wb_motor_set_velocity(rear_right_track, 0.1);
  wb_motor_set_position(front_motor, -1.2);
  wb_motor_set_position(rear_motor, -1.2);

  wb_robot_step(4500);


  wb_motor_set_velocity(front_left_track, 0.0);
  wb_motor_set_velocity(front_right_track, 0.0);
  wb_motor_set_velocity(rear_left_track, 0.0);
  wb_motor_set_velocity(rear_right_track, 0.0);
  wb_motor_set_position(arm_motors[0], -1.51);
  wb_motor_set_position(arm_motors[1], 0.69);
  wb_motor_set_position(arm_motors[2], -0.38);
  wb_motor_set_position(arm_motors[3], -1.63);
  wb_motor_set_position(arm_motors[4], -0.75);
  wb_motor_set_position(arm_motors[5], 0.57);
  wb_motor_set_position(arm_motors[6], 0.69);

  while (true)
    wb_robot_step(TIME_STEP);

  wb_robot_cleanup();

  return 0;
}
