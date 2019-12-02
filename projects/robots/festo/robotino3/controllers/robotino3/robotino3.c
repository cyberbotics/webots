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
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 * Description: Simple avoidance controller
 *              The velocity of each wheel is set according to a
 *              Braitenberg-like algorithm which takes the values returned
 *              by the Hokuyo URG-04LX-UG01 as input.
 */

#include <webots/keyboard.h>
#include <webots/robot.h>

#include <base.h>
#include <tiny_math.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void automatic_behavior() {
  // Make a square movement
  passive_wait(1.0);

  base_forwards();
  passive_wait(2.5);
  base_reset();
  passive_wait(1.0);

  base_strafe_left();
  passive_wait(2.5);
  base_reset();
  passive_wait(1.0);

  base_backwards();
  passive_wait(5.0);
  base_reset();
  passive_wait(1.0);

  base_strafe_right();
  passive_wait(5.0);
  base_reset();
  passive_wait(1.0);

  base_forwards();
  passive_wait(5.0);
  base_reset();
  passive_wait(1.0);

  base_strafe_left();
  passive_wait(2.5);
  base_reset();
  passive_wait(1.0);

  base_backwards();
  passive_wait(2.5);
  base_reset();
  passive_wait(1.0);

  base_turn_left();
  passive_wait(1.0);
  base_reset();

  base_turn_right();
  passive_wait(1.0);
  base_reset();
}

static void display_helper_message() {
  printf("Control commands:\n");
  printf(" Arrows:       Move the robot\n");
  printf(" Page Up/Down: Rotate the robot\n");
  printf(" Space: Reset\n");
}

int main(int argc, char **argv) {

  // Initialization
  wb_robot_init();
  base_init();
  passive_wait(2.0);

  // Demonstration
  if (argc > 1 && strcmp(argv[1], "demo") == 0)
    automatic_behavior();

  // Display instructions to control the robot
  display_helper_message();

  int pc = 0;
  wb_keyboard_enable(TIME_STEP);

  // User moves the robot with keyboard arrows
  while (true) {
    step();

    int c = wb_keyboard_get_key();
    if ((c >= 0) && c != pc) {
      switch (c) {
      case WB_KEYBOARD_UP:
        printf("Go forwards\n");
        base_forwards();
        break;
      case WB_KEYBOARD_DOWN:
        printf("Go backwards\n");
        base_backwards();
        break;
      case WB_KEYBOARD_LEFT:
        printf("Strafe left\n");
        base_strafe_left();
        break;
      case WB_KEYBOARD_RIGHT:
        printf("Strafe right\n");
        base_strafe_right();
        break;
      case WB_KEYBOARD_PAGEUP:
        printf("Turn left\n");
        base_turn_left();
        break;
      case WB_KEYBOARD_PAGEDOWN:
        printf("Turn right\n");
        base_turn_right();
        break;
      case WB_KEYBOARD_END:
      case ' ':
        printf("Reset\n");
        base_reset();
        break;
      default:
        fprintf(stderr, "Wrong keyboard input\n");
        break;
      }
    }
    pc = c;
  }

  wb_robot_cleanup();

  return 0;
}
