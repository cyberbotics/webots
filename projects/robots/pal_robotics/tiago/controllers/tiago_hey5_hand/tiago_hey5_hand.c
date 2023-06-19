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
 * Description: Simple controller to make an "Hello" mouvement with the
 *              hand showing the peace symbol.
 *              It is also possible to move the robot with the keyboard arrows.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAX_SPEED 7.0  // [rad/s]
#define N_PARTS 45
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

  // get devices
  // initialize the robot's information
  const char *names[N_PARTS] = {"head_2_joint",
                                "head_1_joint",
                                "torso_lift_joint",
                                "arm_1_joint",
                                "arm_2_joint",
                                "arm_3_joint",
                                "arm_4_joint",
                                "arm_5_joint",
                                "arm_6_joint",
                                "arm_7_joint",
                                "hand_right_thumb_abd_joint",
                                "hand_right_thumb_virtual_1_joint",
                                "hand_right_thumb_flex_1_joint",
                                "hand_right_thumb_virtual_2_joint",
                                "hand_right_thumb_flex_2_joint",
                                "hand_right_index_abd_joint",
                                "hand_right_index_virtual_1_joint",
                                "hand_right_index_flex_1_joint",
                                "hand_right_index_virtual_2_joint",
                                "hand_right_index_flex_2_joint",
                                "hand_right_index_virtual_3_joint",
                                "hand_right_index_flex_3_joint",
                                "hand_right_middle_abd_joint",
                                "hand_right_middle_virtual_1_joint",
                                "hand_right_middle_flex_1_joint",
                                "hand_right_middle_virtual_2_joint",
                                "hand_right_middle_flex_2_joint",
                                "hand_right_middle_virtual_3_joint",
                                "hand_right_middle_flex_3_joint",
                                "hand_right_ring_abd_joint",
                                "hand_right_ring_virtual_1_joint",
                                "hand_right_ring_flex_1_joint",
                                "hand_right_ring_virtual_2_joint",
                                "hand_right_ring_flex_2_joint",
                                "hand_right_ring_virtual_3_joint",
                                "hand_right_ring_flex_3_joint",
                                "hand_right_little_abd_joint",
                                "hand_right_little_virtual_1_joint",
                                "hand_right_little_flex_1_joint",
                                "hand_right_little_virtual_2_joint",
                                "hand_right_little_flex_2_joint",
                                "hand_right_little_virtual_3_joint",
                                "hand_right_little_flex_3_joint",
                                "wheel_left_joint",
                                "wheel_right_joint"};

  double target_pos[N_PARTS] = {0.24,  -0.67, 0.09, 0.07, 0.26, -3.16, 1.27, 1.32,     0.00,    1.41, 1.55,  0.79,
                                0.68,  0.00,  0.00, 0.00, 0.00, 0.00,  0.00, 0.00,     0.00,    0.00, -0.08, 0.00,
                                0.00,  0.00,  0.00, 0.00, 0.00, -0.22, 0.00, 0.32,     0.79,    0.42, 0.79,  0.79,
                                -0.52, 0.00,  0.79, 0.79, 0.37, 0.79,  0.62, INFINITY, INFINITY};

  // configures and achieves the robot's position desired
  for (int i = 0; i < N_PARTS; i++) {
    robot_parts[i] = wb_robot_get_device(names[i]);
    wb_motor_set_velocity(robot_parts[i], wb_motor_get_max_velocity(robot_parts[i]) / 2.0);
    wb_motor_set_position(robot_parts[i], target_pos[i]);
  }

  // print user instructions
  printf("You can drive this robot by selecting the 3D window and pressing the keyboard arrows.\n");

  // enable keyboard
  wb_keyboard_enable(time_step);

  const double initialTime = wb_robot_get_time();

  while (wb_robot_step(time_step) != -1) {
    check_keyboard();

    // Hello mouvement
    const double time = wb_robot_get_time() - initialTime;
    wb_motor_set_position(robot_parts[8], 0.3 * sin(5.0 * time) - 0.3);
  };

  wb_robot_cleanup();
  return 0;
}
