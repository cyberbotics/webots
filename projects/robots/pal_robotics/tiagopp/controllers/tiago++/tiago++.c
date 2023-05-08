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
 * Description: Simple controller to make an "Hello" mouvement with both arms.
 *              It is also possible to move the robot with the keyboard arrows.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/camera.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>

#define MAX_SPEED 7.0  // [rad/s]
#define N_PARTS 19
#define MOTOR_LEFT N_PARTS - 2
#define MOTOR_RIGHT N_PARTS - 1

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
}

int main(int argc, char **argv) {
  // init webots stuff
  wb_robot_init();

  const int time_step = wb_robot_get_basic_time_step();

  // enable RGBD camera
  WbDeviceTag rgb_camera = wb_robot_get_device("Astra rgb");
  wb_camera_enable(rgb_camera, time_step);
  WbDeviceTag depth_camera = wb_robot_get_device("Astra depth");
  wb_range_finder_enable(depth_camera, time_step);

  // get devices
  // initialize the robot's information
  const char *names[N_PARTS] = {"head_2_joint",      "head_1_joint",      "torso_lift_joint",  "arm_right_1_joint",
                                "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint",
                                "arm_right_6_joint", "arm_right_7_joint", "arm_left_1_joint",  "arm_left_2_joint",
                                "arm_left_3_joint",  "arm_left_4_joint",  "arm_left_5_joint",  "arm_left_6_joint",
                                "arm_left_7_joint",  "wheel_left_joint",  "wheel_right_joint"};

  double target_pos[N_PARTS] = {0.24, -0.67, 0.09, -0.43, -0.77, 0.00, 0.96, 1.41,     1.2,     0.00,
                                0.74, -0.95, 0.06, 1.12,  1.45,  0.00, 0.00, INFINITY, INFINITY};

  // configures and achieves the robot's position desired
  for (int i = 0; i < N_PARTS; i++) {
    robot_parts[i] = wb_robot_get_device(names[i]);
    wb_motor_set_velocity(robot_parts[i], wb_motor_get_max_velocity(robot_parts[i]) / 2.0);
    wb_motor_set_position(robot_parts[i], target_pos[i]);
  }

  // print user instructions
  printf("You can drive this robot by selecting the 3D window and pressing the keyboard arrows.\n");

  // enable keyboard and start forward motion
  wb_keyboard_enable(time_step);

  const double initialTime = wb_robot_get_time();

  while (wb_robot_step(time_step) != -1) {
    check_keyboard();

    // Hello mouvement
    const double time = wb_robot_get_time() - initialTime;
    wb_motor_set_position(robot_parts[6], 0.4 * sin(2.0 * time) + 1.17);   // arm_right_4_joint
    wb_motor_set_position(robot_parts[8], 0.3 * sin(5.0 * time));          // arm_right_6_joint
    wb_motor_set_position(robot_parts[13], 0.4 * sin(2.0 * time) + 1.17);  // arm_left_4_joint
    wb_motor_set_position(robot_parts[15], 0.3 * sin(5.0 * time));         // arm_left_6_joint
  };

  wb_robot_cleanup();
  return 0;
}
