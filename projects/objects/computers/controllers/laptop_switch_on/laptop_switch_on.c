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
 * Description:   Switch on the laptop PROTO
 */

#include <stdio.h>
#include <webots/display.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag screen_display = wb_robot_get_device("display");
  WbImageRef screenImage = wb_display_image_load(screen_display, "screen_display.png");
  WbDeviceTag position_sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  wb_display_set_color(screen_display, 0);
  while (wb_robot_step(TIME_STEP) != -1) {
    double position = wb_position_sensor_get_value(position_sensor);

    if (position > -0.8)
      wb_display_image_paste(screen_display, screenImage, 0, 0, false);
    else
      wb_display_fill_rectangle(screen_display, 0, 0, 620, 400);
  };

  wb_robot_cleanup();

  return 0;
}
