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
 * Description:   An example of a controller using the Track node to move
 *                tracked robot.
 */

#include <stdio.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

static void print_keyboard_help() {
  printf("Select the 3D window and use the keyboard:\n");
  printf("\n");
  printf(" W: forward\n");
  printf(" A: turn left\n");
  printf(" S: backward\n");
  printf(" D: turn right\n");
  printf("\n");
  printf("-------------------------------------------------\n");
}

int main(int argc, char **argv) {
  wb_robot_init();
  const int timeStep = wb_robot_get_basic_time_step();
  const float speed = 0.05;
  wb_keyboard_enable(timeStep);

  WbDeviceTag leftMotor = wb_robot_get_device("left motor");
  WbDeviceTag rightMotor = wb_robot_get_device("right motor");
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);
  wb_motor_set_velocity(leftMotor, 0);
  wb_motor_set_velocity(rightMotor, 0);

  print_keyboard_help();

  while (wb_robot_step(timeStep) != -1) {
    const int key = wb_keyboard_get_key();
    if (key != -1) {
      switch (key) {
        case 'W':
          wb_motor_set_velocity(leftMotor, speed);
          wb_motor_set_velocity(rightMotor, speed);
          break;
        case 'A':
          wb_motor_set_velocity(leftMotor, -speed);
          wb_motor_set_velocity(rightMotor, speed);
          break;
        case 'S':
          wb_motor_set_velocity(leftMotor, -speed);
          wb_motor_set_velocity(rightMotor, -speed);
          break;
        case 'D':
          wb_motor_set_velocity(leftMotor, speed);
          wb_motor_set_velocity(rightMotor, -speed);
          break;
      }
    }
  }

  wb_robot_cleanup();

  return 0;
}
