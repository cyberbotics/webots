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
 * Description:   Switch on a LED for 5 sec when activating the bumper
 */

#include <webots/led.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include <stdlib.h>

#define TIME_STEP 20

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void wait_second(double time) {
  double start_time = wb_robot_get_time();
  while (start_time + time > wb_robot_get_time())
    step();
}

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag led = wb_robot_get_device("led");
  WbDeviceTag bumper = wb_robot_get_device("bumper");
  wb_touch_sensor_enable(bumper, TIME_STEP);

  while (true) {
    step();

    bool bumped = wb_touch_sensor_get_value(bumper);

    if (bumped) {
      wb_led_set(led, true);
      wait_second(5.0);
    } else
      wb_led_set(led, false);
  };

  return EXIT_SUCCESS;
}
