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
 * Description:   Generic traffic light controller
 * Arguments: 1) red time seconds (double, default 20s)
 *            2) green time seconds (double, if only one time is set, use same for booth)
 *            3) start color (r: red, g: green, og: orange to green, or: orange to red)
 */

#include <webots/led.h>
#include <webots/robot.h>

#include <stdio.h>
#include <string.h>

#define TIME_STEP 512
enum { GREEN_STATE, RED_STATE, ORANGE_STATE_TO_RED, ORANGE_STATE_TO_GREEN };

int main(int argc, char **argv) {
  wb_robot_init();
  double red_time = 20.0;
  double green_time = 20.0;
  double orange_time = 1.5;
  int current_state = GREEN_STATE;

  if (argc > 1) {
    sscanf(argv[1], "%lf", &red_time);
    if (argc > 2) {
      sscanf(argv[2], "%lf", &green_time);
      if (argc > 3) {
        if (strcmp(argv[3], "r") == 0)
          current_state = RED_STATE;
        else if (strcmp(argv[3], "g") == 0)
          current_state = GREEN_STATE;
        else if (strcmp(argv[3], "og") == 0)
          current_state = ORANGE_STATE_TO_GREEN;
        else if (strcmp(argv[3], "or") == 0)
          current_state = ORANGE_STATE_TO_RED;
      }
    } else
      green_time = red_time;
  }

  WbDeviceTag red_light, orange_light, green_light;
  red_light = wb_robot_get_device("red light");
  orange_light = wb_robot_get_device("orange light");
  green_light = wb_robot_get_device("green light");
  double last_phase_change_time = 0.0;

  if (current_state == GREEN_STATE) {
    wb_led_set(green_light, 1);
    wb_robot_set_custom_data("green");
  } else if (current_state == RED_STATE) {
    wb_led_set(red_light, 1);
    wb_robot_set_custom_data("red");
  } else {
    wb_led_set(orange_light, 1);
    wb_robot_set_custom_data("orange");
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    double current_time = wb_robot_get_time();

    if (current_state == GREEN_STATE) {
      if ((current_time - last_phase_change_time) >= green_time) {
        current_state = ORANGE_STATE_TO_RED;
        last_phase_change_time = current_time;
        wb_led_set(green_light, 0);
        wb_led_set(orange_light, 1);
        wb_robot_set_custom_data("orange");
      }
    } else if (current_state == RED_STATE) {
      if ((current_time - last_phase_change_time) >= red_time) {
        current_state = ORANGE_STATE_TO_GREEN;
        last_phase_change_time = current_time;
        wb_led_set(red_light, 0);
        wb_led_set(orange_light, 1);
        wb_robot_set_custom_data("orange");
      }
    } else if (current_state == ORANGE_STATE_TO_RED) {
      if ((current_time - last_phase_change_time) >= orange_time) {
        current_state = RED_STATE;
        last_phase_change_time = current_time;
        wb_led_set(orange_light, 0);
        wb_led_set(red_light, 1);
        wb_robot_set_custom_data("red");
      }
    } else {  // current_state == ORANGE_STATE_TO_GREEN
      if ((current_time - last_phase_change_time) >= orange_time) {
        current_state = GREEN_STATE;
        last_phase_change_time = current_time;
        wb_led_set(orange_light, 0);
        wb_led_set(green_light, 1);
        wb_robot_set_custom_data("green");
      }
    }
  };

  wb_robot_cleanup();

  return 0;
}
