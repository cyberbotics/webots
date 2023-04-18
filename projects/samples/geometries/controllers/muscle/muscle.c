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

#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <string.h>

#define TIME_STEP 16

int main(int argc, char **argv) {
  wb_robot_init();
  WbDeviceTag muscle = wb_robot_get_device("muscle");
  WbDeviceTag muscle2 = 0;
  WbDeviceTag ps = wb_robot_get_device("position sensor");
  double p = 0.0;
  double max = 2.0;
  double step = 0.4;
  if (strcmp(wb_robot_get_name(), "slider") == 0) {
    max = 0.08;
    step = 0.05;
  } else if (strcmp(wb_robot_get_name(), "hinge2") == 0) {
    muscle2 = wb_robot_get_device("muscle2");
  }
  double dp = step;
  wb_position_sensor_enable(ps, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    wb_motor_set_position(muscle, p);
    if (muscle2)
      wb_motor_set_position(muscle2, 2 - p);
    const double pos = wb_position_sensor_get_value(ps);
    if (pos <= 0.0)
      dp = -step;
    else if (pos >= max)
      dp = step;
    p = pos - dp;
  };

  wb_robot_cleanup();

  return 0;
}
