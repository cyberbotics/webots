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
 * Description: This supervisor tracks the absolute position of the robot
 *              and displays the result on a Display device.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/display.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/utils/system.h>

#define TIME_STEP 64

#define GROUND_X 1.0
#define GROUND_Y 1.0

#define LIGHT_GRAY 0x505050
#define RED 0xBB2222
#define GREEN 0x22BB11
#define BLUE 0x2222BB

// main function
int main() {
  // init Webtos stuff
  wb_robot_init();

  // First we get a handler to devices
  WbDeviceTag ground_display = wb_robot_get_device("ground_display");

  // get the properties of the Display
  int width = wb_display_get_width(ground_display);
  int height = wb_display_get_height(ground_display);

  // prepare stuff to get the
  // ROBOT(MYBOT).translation field
  WbNodeRef mybot = wb_supervisor_node_get_from_def("MYBOT");
  WbFieldRef translationField = wb_supervisor_node_get_field(mybot, "translation");

  // paint the display's background
  wb_display_set_color(ground_display, LIGHT_GRAY);
  wb_display_fill_rectangle(ground_display, 0, 0, width, height);
  wb_display_set_color(ground_display, RED);
  wb_display_draw_line(ground_display, width / 2, 0, width / 2, height - 1);
  wb_display_draw_text(ground_display, "x", width / 2 - 10, height - 10);
  wb_display_set_color(ground_display, GREEN);
  wb_display_draw_line(ground_display, 0, height / 2, width - 1, height / 2);
  wb_display_draw_text(ground_display, "y", width - 10, height / 2 - 10);

  // init image ref used to save into the image file
  WbImageRef to_store = NULL;

  // init a variable which counts the time steps
  int counter = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    // Update the translation field
    const double *translation = wb_supervisor_field_get_sf_vec3f(translationField);

    // Update the counter
    counter++;

    // display the robot position
    wb_display_set_opacity(ground_display, 0.03);
    wb_display_set_color(ground_display, BLUE);
    wb_display_fill_oval(ground_display, width - width * (translation[1] + GROUND_X / 2) / GROUND_X,
                         height - height * (translation[0] + GROUND_Y / 2) / GROUND_Y, 4, 4);
    // Clear previous to_store
    if (to_store) {
      wb_display_image_delete(ground_display, to_store);
      to_store = NULL;
    }

    // Every 50 steps, store the resulted image into a file
    if (counter % 50 == 0) {
      to_store = wb_display_image_copy(ground_display, 0, 0, width, height);
      // compute the path to store the image in user directory
      char *filepath;
#ifdef _WIN32
      const char *user_directory = wbu_system_short_path(wbu_system_getenv("USERPROFILE"));
      filepath = (char *)malloc(strlen(user_directory) + 16);
      strcpy(filepath, user_directory);
      strcat(filepath, "\\screenshot.png");
#else
      const char *user_directory = wbu_system_getenv("HOME");
      filepath = (char *)malloc(strlen(user_directory) + 16);
      strcpy(filepath, user_directory);
      strcat(filepath, "/screenshot.png");
#endif
      wb_display_image_save(ground_display, to_store, filepath);
      free(filepath);
    }
  }

  wb_robot_cleanup();

  return 0;
}
