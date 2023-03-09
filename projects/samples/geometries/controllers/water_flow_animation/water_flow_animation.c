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
 * Description:  animate water flow
 */

#include <stdio.h>
#include <stdlib.h>  // exit and EXIT_SUCCESS

#include <webots/display.h>
#include <webots/robot.h>

#define TIME_STEP 32
#define WATER_TILE_HEIGHT 512

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

int main(int argc, char **argv) {
  wb_robot_init();

  // get the devices
  WbDeviceTag lower_display = wb_robot_get_device("lower display");
  WbDeviceTag upper_display = wb_robot_get_device("upper display");

  // load the image
  WbImageRef lower_image = wb_display_image_load(lower_display, "water_flow.jpg");
  WbImageRef upper_image = wb_display_image_load(upper_display, "water_flow.jpg");

  // main loop
  int counter = 0;
  const int increment = 14;
  const int number_of_images = WATER_TILE_HEIGHT / increment;
  while (true) {
    counter %= number_of_images;
    const int shift = -WATER_TILE_HEIGHT + counter * increment;
    wb_display_image_paste(lower_display, lower_image, 0, shift, true);
    wb_display_image_paste(upper_display, upper_image, 0, shift, true);
    ++counter;
    step();
  }

  return EXIT_SUCCESS;
}
