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

/*
 * Description: Simple controller to make an "Hello" mouvement.
 *              It is also possible to move the robot with the keyboard arrows.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 8
#define MAX_SPEED 7.0  // [rad/s]
#define MOTOR_LEFT 10
#define MOTOR_RIGHT 11
#define N_PARTS 12

enum XYZAComponents { X, Y, Z, ALPHA };

static int old_key = -1;
static WbDeviceTag robot_parts[N_PARTS];

static void check_keyboard() {
  double speeds_left = 0.0, speeds_right = 0.0;

  int key = wb_keyboard_get_key();
  if (key >= 0) {
    switch (key) {
      case WB_KEYBOARD_UP:
        speeds_left = MAX_SPEED;
        speeds_right = MAX_SPEED;
        break;
      case WB_KEYBOARD_DOWN:
        speeds_left = -MAX_SPEED;
        speeds_right = -MAX_SPEED;
        break;
      case WB_KEYBOARD_RIGHT:
        speeds_left = MAX_SPEED;
        speeds_right = -MAX_SPEED;
        break;
      case WB_KEYBOARD_LEFT:
        speeds_left = -MAX_SPEED;
        speeds_right = MAX_SPEED;
        break;
    }
  }

  wb_motor_set_velocity(robot_parts[MOTOR_LEFT], speeds_left);
  wb_motor_set_velocity(robot_parts[MOTOR_RIGHT], speeds_right);
  old_key = key;
}

int main(int argc, char **argv) {
  // init webots stuff
  wb_robot_init();

  // get devices
  // initialize the robot's information
  const char *names[N_PARTS] = {"head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
                                "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
                                "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint"};

  double target_pos[N_PARTS] = {0.24, -0.67, 0.09, 0.07, 0.26, -3.16, 1.27, 1.32, 0.0, 1.41, INFINITY, INFINITY};

  double robot_parts_Vmax[N_PARTS] = {0.24, -0.67, 0.09, 0.07, 0.26, -3.16, 1.27, 1.32, 0.0, 1.41, MAX_SPEED, MAX_SPEED};

  // configures and achieves the robot's position desired
  for (int i = 0; i < N_PARTS; i++) {
    robot_parts[i] = wb_robot_get_device(names[i]);
    robot_parts_Vmax[i] = wb_motor_get_max_velocity(robot_parts[i]);
    wb_motor_set_velocity(robot_parts[i], robot_parts_Vmax[i] / 2.0);
    wb_motor_set_position(robot_parts[i], target_pos[i]);
  }

  // print user instructions
  printf("\f");
  printf("You can drive this robot:\n");
  printf("Select the 3D window and use cursor keys:\n");
  printf("\n");

  // enable keyboard and start forward motion
  wb_keyboard_enable(TIME_STEP);
  wb_motor_set_velocity(robot_parts[MOTOR_LEFT], MAX_SPEED / 2.0);   // left
  wb_motor_set_velocity(robot_parts[MOTOR_RIGHT], MAX_SPEED / 2.0);  // right

  double initTime = wb_robot_get_time();

  while (wb_robot_step(TIME_STEP) != -1) {
    check_keyboard();

    // Hello mouvement
    double time = wb_robot_get_time() - initTime;
    wb_motor_set_position(robot_parts[8], 0.3 * sin(5 * time) - 0.3);
  };

  wb_robot_cleanup();
  return 0;
}
