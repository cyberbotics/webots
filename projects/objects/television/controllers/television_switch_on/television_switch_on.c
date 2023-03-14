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
 * Description:  Switch on the television prototype
 */

#include <stdio.h>
#include <stdlib.h>  // exit and EXIT_SUCCESS

#include <webots/display.h>
#include <webots/led.h>
#include <webots/robot.h>

#define TIME_STEP 64

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

int main(int argc, char **argv) {
  wb_robot_init();

  printf("Default controller of the television started...\n");

  // get the devices
  WbDeviceTag led = wb_robot_get_device("led");
  WbDeviceTag display = wb_robot_get_device("display");

  // switch on the led
  wb_led_set(led, 1);

  // load the image
  WbImageRef movie = wb_display_image_load(display, "movie.png");

  // main loop
  int counter = 0;
  while (true) {
    // display the next frame each step
    switch (counter % 4) {
      case 0:
        wb_display_image_paste(display, movie, 0, 0, false);
        break;
      case 1:
        wb_display_image_paste(display, movie, -128, 0, false);
        break;
      case 2:
        wb_display_image_paste(display, movie, 0, -64, false);
        break;
      default:
        wb_display_image_paste(display, movie, -128, -64, false);
        break;
    }

    // update the counter
    counter++;

    // step
    step();
  }

  return EXIT_SUCCESS;
}
