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

#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <stdlib.h>

#include "sc_control.h"

#define TIME_STEP 64
#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 333.0  // 500.0 * 2 / 3
#define TURN_SPEED (0.5 * MAX_SPEED)
#define SPEED_UNIT 0.00628

static void startup_message() {
  printf("Look at the anaglyph stereo image with cyan-red 3D glasses.\n");
  printf("Drive the iRobot Create robot with the keyboard arrows.\n");
}

int main(int argc, char **argv) {
  wb_robot_init();

  startup_message();

  // get a handler to the motors and set target position to infinity (speed control)
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  bool success = sc_init(TIME_STEP, true, 15, 2.5);
  if (!success) {
    fprintf(stderr, "Cannot init the stereoscopic camera\n");
    wb_robot_cleanup();
    exit(EXIT_FAILURE);
  }

  wb_keyboard_enable(TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    sc_update();

    double speeds[2] = {0.0, 0.0};
    int c = wb_keyboard_get_key();
    while (c >= 0) {
      switch (c) {
        case WB_KEYBOARD_UP:
          speeds[LEFT] += MAX_SPEED;
          speeds[RIGHT] += MAX_SPEED;
          break;
        case WB_KEYBOARD_DOWN:
          speeds[LEFT] -= MAX_SPEED;
          speeds[RIGHT] -= MAX_SPEED;
          break;
        case WB_KEYBOARD_LEFT:
          speeds[LEFT] -= TURN_SPEED;
          speeds[RIGHT] += TURN_SPEED;
          break;
        case WB_KEYBOARD_RIGHT:
          speeds[LEFT] += TURN_SPEED;
          speeds[RIGHT] -= TURN_SPEED;
          break;
      }
      c = wb_keyboard_get_key();
    }
    wb_motor_set_velocity(left_motor, SPEED_UNIT * speeds[LEFT]);
    wb_motor_set_velocity(right_motor, SPEED_UNIT * speeds[RIGHT]);
  }

  sc_cleanup();
  wb_robot_cleanup();

  return 0;
}
