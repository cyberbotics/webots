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
 * Description:   Simple controller simulating a defective street light
 */

#include <webots/led.h>
#include <webots/robot.h>

#define TIME_STEP 50

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag led = wb_robot_get_device("led");
  int step_to_perform = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    if (step_to_perform <= 0) {
      int led_state = wb_led_get(led);
      if (led_state == 255) {
        // led is on, switch it off for 20steps (1s)
        step_to_perform = 20;
        wb_led_set(led, 0);
      } else if (led_state == 170) {
        // led is switching on (2/3 power) switch it fully on for 60steps (3s)
        step_to_perform = 60;
        wb_led_set(led, 255);
      } else if (led_state == 85) {
        // led is switching on (1/3 power) continue switch on for 2steps (0.1s)
        step_to_perform = 2;
        wb_led_set(led, 170);
      } else {
        // led is off, try to switch on (1/3 power) for 2steps (0.1s)
        step_to_perform = 2;
        wb_led_set(led, 85);
      }
    } else
      step_to_perform--;
  };

  wb_robot_cleanup();

  return 0;
}
