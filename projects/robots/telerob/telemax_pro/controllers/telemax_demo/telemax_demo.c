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
#include <stdlib.h>

#define TIME_STEP 32

void noop(int duration) {
  // Wait without applying any command during 'duration' milliseconds.
  int counter = 0;
  while (counter < duration) {
    counter += TIME_STEP;
    if (wb_robot_step(TIME_STEP) == -1) {
      wb_robot_cleanup();
      exit(0);
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
  WbDeviceTag camera_led = wb_robot_get_device("camera 0 led");

  wb_camera_enable(camera, TIME_STEP);
  wb_camera_enable(gripper_camera, TIME_STEP);
  wb_led_set(camera_led, 60);

  // go out of the box
  wb_motor_set_position(front_left_track, INFINITY);
  wb_motor_set_position(front_right_track, INFINITY);
  wb_motor_set_position(rear_left_track, INFINITY);
  wb_motor_set_position(rear_right_track, INFINITY);
  wb_motor_set_velocity(front_left_track, -0.3);
  wb_motor_set_velocity(front_right_track, -0.3);
  wb_motor_set_velocity(rear_left_track, 0.3);
  wb_motor_set_velocity(rear_right_track, 0.3);
  noop(7000);

  // prepare tracks for stairs
  wb_motor_set_position(camera_tilt, -0.1);
  wb_motor_set_velocity(front_left_track, 0.02);
  wb_motor_set_velocity(front_right_track, 0.02);
  wb_motor_set_velocity(rear_left_track, 0.02);
  wb_motor_set_velocity(rear_right_track, 0.02);
  wb_motor_set_position(front_motor, 1.0);
  wb_motor_set_position(rear_motor, -0.2);
  noop(7000);

  // climb stairs
  wb_motor_set_velocity(front_left_track, -0.1);
  wb_motor_set_velocity(front_right_track, -0.1);
  wb_motor_set_velocity(rear_left_track, 0.1);
  wb_motor_set_velocity(rear_right_track, 0.1);
  noop(13000);

  // align fron tracks with stairs
  wb_motor_set_position(front_motor, 0.25);

  // look around with camera
  wb_motor_set_position(camera_pan, 1.0);
  noop(2000);
  wb_motor_set_position(camera_pan, -1.0);
  noop(4000);
  wb_motor_set_position(camera_pan, 0.0);
  noop(2000);
  // camera zoom
  while (wb_camera_get_fov(camera) > 0.2) {
    wb_camera_set_fov(camera, 0.99 * wb_camera_get_fov(camera));
    noop(16);
  }
  while (wb_camera_get_fov(camera) < 0.78) {
    wb_camera_set_fov(camera, 1.01 * wb_camera_get_fov(camera));
    noop(16);
  }
  wb_motor_set_position(camera_tilt, -0.4);
  noop(2000);
  wb_motor_set_position(camera_tilt, 0.5);
  noop(40000);

  // put tracks in 'flat' position
  wb_motor_set_position(front_motor, 0.0);
  wb_motor_set_position(rear_motor, 0.0);
  wb_motor_set_position(camera_tilt, 0.0);
  noop(3000);

  // go in 'high' position
  wb_motor_set_velocity(front_left_track, 0.1);
  wb_motor_set_velocity(front_right_track, 0.1);
  wb_motor_set_velocity(rear_left_track, 0.1);
  wb_motor_set_velocity(rear_right_track, 0.1);
  wb_motor_set_position(front_motor, -1.2);
  wb_motor_set_position(rear_motor, -1.2);
  noop(4500);

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
  noop(5000);

  // actuate gripper
  for (i = 0; i < 3; ++i) {
    wb_motor_set_position(left_gripper, 0.012);
    wb_motor_set_position(right_gripper, 0.012);
    noop(1000);
    wb_motor_set_position(left_gripper, -0.005);
    wb_motor_set_position(right_gripper, -0.005);
    noop(1000);
  }

  // push the door
  wb_motor_set_velocity(front_left_track, -0.1);
  wb_motor_set_velocity(front_right_track, -0.1);
  wb_motor_set_velocity(rear_left_track, 0.1);
  wb_motor_set_velocity(rear_right_track, 0.1);
  noop(10000);

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
