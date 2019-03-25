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

void wait(int time) {
  int counter = 0;
  while (counter < time) {
    counter += TIME_STEP;
    if (wb_robot_step(TIME_STEP) == -1) {
      wb_robot_cleanup();
      return 0;
    }
  }
}

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

  WbDeviceTag left_gripper = wb_robot_get_device("left gripper motor");
  WbDeviceTag right_gripper = wb_robot_get_device("right gripper motor");
  WbDeviceTag gripper_camera = wb_robot_get_device("gripper camera");

  WbDeviceTag camera_pan = wb_robot_get_device("camera 0 pan motor");
  WbDeviceTag camera_tilt = wb_robot_get_device("camera 0 tilt motor");
  WbDeviceTag camera = wb_robot_get_device("camera 0");

  // go out of the box
  wb_motor_set_position(front_left_track, INFINITY);
  wb_motor_set_position(front_right_track, INFINITY);
  wb_motor_set_position(rear_left_track, INFINITY);
  wb_motor_set_position(rear_right_track, INFINITY);
  wb_motor_set_velocity(front_left_track, -0.1);
  wb_motor_set_velocity(front_right_track, -0.1);
  wb_motor_set_velocity(rear_left_track, 0.1);
  wb_motor_set_velocity(rear_right_track, 0.1);

  wait(20000);

  // look around with camera
  wb_motor_set_velocity(front_left_track, 0.0);
  wb_motor_set_velocity(front_right_track, 0.0);
  wb_motor_set_velocity(rear_left_track, 0.0);
  wb_motor_set_velocity(rear_right_track, 0.0);
  wb_camera_enable(camera, TIME_STEP);
  wb_motor_set_position(camera_pan, 1.0);
  wait(2000);
  wb_motor_set_position(camera_pan, -1.0);
  wait(4000);
  wb_motor_set_position(camera_pan, 0.0);
  wait(2000);
  wb_motor_set_position(camera_tilt, -0.5);
  wait(2000);
  wb_motor_set_position(camera_tilt, 0.5);
  wait(4000);
  wb_motor_set_position(camera_tilt, -0.1);

  // prepare tracks for stairs
  wb_motor_set_position(front_motor, 1.0);
  wb_motor_set_position(rear_motor, -0.2);

  wait(7000);

  // climb stairs
  wb_motor_set_velocity(front_left_track, -0.1);
  wb_motor_set_velocity(front_right_track, -0.1);
  wb_motor_set_velocity(rear_left_track, 0.1);
  wb_motor_set_velocity(rear_right_track, 0.1);

  wait(13000);

  // align fron tracks with stairs
  wb_motor_set_position(front_motor, 0.25);

  wait(60000);

  // put tracks in 'flat' position
  wb_motor_set_position(front_motor, 0.0);
  wb_motor_set_position(rear_motor, 0.0);

  wait(5000);

  // go in 'high' position
  wb_motor_set_velocity(front_left_track, 0.1);
  wb_motor_set_velocity(front_right_track, 0.1);
  wb_motor_set_velocity(rear_left_track, 0.1);
  wb_motor_set_velocity(rear_right_track, 0.1);
  wb_motor_set_position(front_motor, -1.2);
  wb_motor_set_position(rear_motor, -1.2);

  wait(4500);

  // stop tracks and move arm
  wb_motor_set_velocity(front_left_track, 0.0);
  wb_motor_set_velocity(front_right_track, 0.0);
  wb_motor_set_velocity(rear_left_track, 0.0);
  wb_motor_set_velocity(rear_right_track, 0.0);
  wb_motor_set_position(arm_motors[0], 0.0);
  wb_motor_set_position(arm_motors[1], 0.7);
  wb_motor_set_position(arm_motors[2], 0.0);
  wb_motor_set_position(arm_motors[3], -1.45);
  wb_motor_set_position(arm_motors[4], -0.0);
  wb_motor_set_position(arm_motors[5], -0.75);
  wb_motor_set_position(arm_motors[6], 0.0);

  wait(5000);

  // actuate gripper
  wb_camera_enable(gripper_camera, TIME_STEP);
  for (i = 0; i < 3; ++i) {
    wb_motor_set_position(left_gripper, 0.012);
    wb_motor_set_position(right_gripper, 0.012);
    wait(1000);
    wb_motor_set_position(left_gripper, -0.005);
    wb_motor_set_position(right_gripper, -0.005);
    wait(1000);
  }

  // push the door
  wb_motor_set_velocity(front_left_track, -0.1);
  wb_motor_set_velocity(front_right_track, -0.1);
  wb_motor_set_velocity(rear_left_track, 0.1);
  wb_motor_set_velocity(rear_right_track, 0.1);

  wait(10000);

  // rotate gripper camera to see in the other room
  wb_motor_set_velocity(front_left_track, 0.0);
  wb_motor_set_velocity(front_right_track, 0.0);
  wb_motor_set_velocity(rear_left_track, 0.0);
  wb_motor_set_velocity(rear_right_track, 0.0);
  wb_motor_set_position(arm_motors[4], 1.38);
  wb_motor_set_position(arm_motors[5], 0.31);
  wb_motor_set_position(arm_motors[6], -0.75);

  wb_robot_cleanup();

  return 0;
}
