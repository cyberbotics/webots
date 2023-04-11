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
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <stdlib.h>

// Macros to determine min or max between 2 inputs.
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

int main(int argc, char **argv) {
  wb_robot_init();

  const int timestep = (int)wb_robot_get_basic_time_step();
  wb_keyboard_enable(timestep);

  printf("Control the grabber base with the computer keyboard:\n");
  printf("- Up/Down arrow: move first pivot.\n");
  printf("- Ctrl + Up/Down arrow: move second pivot.\n");
  printf("- Left/Right arrow: move twister.\n");
  printf("- Ctrl + Left/Right arrow: grab or release fingers.\n");

  // Get the motors.
  WbDeviceTag twister = wb_robot_get_device("twister");
  WbDeviceTag pivot_a = wb_robot_get_device("pivot A");
  WbDeviceTag pivot_b = wb_robot_get_device("pivot B");
  WbDeviceTag finger_a = wb_robot_get_device("grabber finger A");
  WbDeviceTag finger_b = wb_robot_get_device("grabber finger B");
  WbDeviceTag finger_c = wb_robot_get_device("grabber finger C");

  // Set LED colors.
  wb_led_set(wb_robot_get_device("twister led"), 0xFF0000);
  wb_led_set(wb_robot_get_device("pivot A led"), 0x00FF00);
  wb_led_set(wb_robot_get_device("pivot B led"), 0x0000FF);
  wb_led_set(wb_robot_get_device("grabber led"), 0x00FFFF);

  // Motor position variables.
  double t = 0.0;
  double p_a = 0.0;
  double p_b = 0.0;
  double g = 0.1;

  // Loop until the simulator stops the controller.
  while (wb_robot_step(timestep) != -1) {
    // Update the motor positions depending on the keyboard input.
    int key = 0;
    do {
      key = wb_keyboard_get_key();
      if (key == WB_KEYBOARD_DOWN)
        p_a = MAX(wb_motor_get_min_position(pivot_a), p_a - 0.01);
      else if (key == WB_KEYBOARD_UP)
        p_a = MIN(wb_motor_get_max_position(pivot_a), p_a + 0.01);
      else if (key == WB_KEYBOARD_RIGHT)
        t = MAX(wb_motor_get_min_position(twister), t - 0.01);
      else if (key == WB_KEYBOARD_LEFT)
        t = MIN(wb_motor_get_max_position(twister), t + 0.01);
      else if (key == WB_KEYBOARD_CONTROL + WB_KEYBOARD_DOWN)
        p_b = MAX(wb_motor_get_min_position(pivot_b), p_b - 0.01);
      else if (key == WB_KEYBOARD_CONTROL + WB_KEYBOARD_UP)
        p_b = MIN(wb_motor_get_max_position(pivot_b), p_b + 0.01);
      else if (key == WB_KEYBOARD_CONTROL + WB_KEYBOARD_LEFT)
        g = MAX(0.1, g - 0.01);
      else if (key == WB_KEYBOARD_CONTROL + WB_KEYBOARD_RIGHT)
        g = MIN(wb_motor_get_max_position(finger_a), g + 0.01);
    } while (key > 0);

    // Actually set the motor positions.
    wb_motor_set_position(twister, t);
    wb_motor_set_position(pivot_a, p_a);
    wb_motor_set_position(pivot_b, p_b);
    wb_motor_set_position(finger_a, g);
    wb_motor_set_position(finger_b, g);
    wb_motor_set_position(finger_c, g);
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
