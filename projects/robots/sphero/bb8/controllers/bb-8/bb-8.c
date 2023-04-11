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

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <stdio.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

typedef enum { AUTOPILOT, MANUAL } Mode;

void usage() {
  printf("Sample controller of the Sphero's BB-8 robot.\n");
  printf("\n");
  printf("The auto-pilot feature is enabled.\n");
  printf("You can pilot the robot by selecting it, and using the computer keyboard:\n");
  printf("- 'up/down' arrow keys:    move the BB-8 forward or backwards.\n");
  printf("- 'left/right' arrow keys: spin the BB-8 to the left or to the right.\n");
  printf("- 'space' key:             stop the BB-8 motors.\n");
  printf("- 'A' key:                 toggle on/off the auto-pilot mode.");
}

int main(int argc, char **argv) {
  wb_robot_init();

  usage();

  int time_step = (int)wb_robot_get_basic_time_step();

  // init the motors in control by velocity
  WbDeviceTag body_yaw_motor = wb_robot_get_device("body yaw motor");
  wb_motor_set_position(body_yaw_motor, INFINITY);
  wb_motor_set_velocity(body_yaw_motor, 0.0);

  WbDeviceTag body_pitch_motor = wb_robot_get_device("body pitch motor");
  wb_motor_set_position(body_pitch_motor, INFINITY);
  wb_motor_set_velocity(body_pitch_motor, 0.0);

  WbDeviceTag head_yaw_motor = wb_robot_get_device("head yaw motor");
  wb_motor_set_position(head_yaw_motor, INFINITY);
  wb_motor_set_velocity(head_yaw_motor, 0.0);

  // enable the camera if it is present on the robot
  int n_devices = wb_robot_get_number_of_devices();
  int i;
  for (i = 0; i < n_devices; ++i) {
    WbDeviceTag device = wb_robot_get_device_by_index(i);

    if (wb_device_get_node_type(device) == WB_NODE_CAMERA)
      wb_camera_enable(device, time_step);
  }

  // enable the computer keyboard
  wb_keyboard_enable(time_step);

  // mode
  Mode mode = AUTOPILOT;

  // speeds
  double yaw_speed = 0.0;
  double pitch_speed = 0.0;
  const double max_speed = 4.0;
  const double attenuation = 0.9;

  // main loop
  while (wb_robot_step(time_step) != -1) {
    // manual mode
    int c;
    do {
      c = wb_keyboard_get_key();
      if (c == -1)
        break;

      if (mode == AUTOPILOT) {
        yaw_speed = 0.0;
        pitch_speed = 0.0;
      }
      mode = MANUAL;

      switch (c) {
        case 'A':
          mode = AUTOPILOT;
          break;
        case WB_KEYBOARD_UP:
          pitch_speed += attenuation;
          break;
        case WB_KEYBOARD_DOWN:
          pitch_speed -= attenuation;
          break;
        case WB_KEYBOARD_RIGHT:
          yaw_speed -= attenuation;
          break;
        case WB_KEYBOARD_LEFT:
          yaw_speed += attenuation;
          break;
        case ' ':
          yaw_speed = 0.0;
          pitch_speed = 0.0;
          break;
      }
    } while (c);

    // speed attenuation
    pitch_speed = MIN(max_speed, MAX(-max_speed, attenuation * pitch_speed));
    yaw_speed = MIN(max_speed, MAX(-max_speed, attenuation * yaw_speed));

    // autopilot mode
    if (mode == AUTOPILOT) {
      double t = wb_robot_get_time();
      if (t > 1.0) {
        yaw_speed = 1.0 * sin(5.0 * t / 6.24);
        pitch_speed = 4.0;
      }
    }

    // set the motor speeds
    wb_motor_set_velocity(body_yaw_motor, yaw_speed);
    wb_motor_set_velocity(head_yaw_motor, yaw_speed);
    wb_motor_set_velocity(body_pitch_motor, pitch_speed);
  };

  wb_robot_cleanup();

  return 0;
}
