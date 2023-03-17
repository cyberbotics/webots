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
 * Description:  This supervisor tracks down the absolute position of the robot
 *               and removes the dust from the area covered by the robot.
 */

#include <stdlib.h>
#include <webots/display.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 64

#define X 0
#define Y 1
#define Z 2

// size of the ground
#define GROUND_X 9.9
#define GROUND_Y 9.9

// main function
int main() {
  // init Webtos stuff
  wb_robot_init();

  // First we get a handler to devices
  WbDeviceTag display = wb_robot_get_device("ground_display");

  // get the properties of the Display
  int width = wb_display_get_width(display);
  int height = wb_display_get_height(display);

  // prepare stuff to get the
  // Robot(IROBOT_CREATE).translation field
  WbNodeRef mybot = wb_supervisor_node_get_from_def("IROBOT_CREATE");
  WbFieldRef translationField = wb_supervisor_node_get_field(mybot, "translation");

  // set the background (otherwise an empty ground is displayed at this step)
  WbImageRef background = wb_display_image_load(display, "dust.jpg");
  wb_display_image_paste(display, background, 0, 0, false);

  // set the pen to remove the texture
  wb_display_set_alpha(display, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    // Update the translation field
    const double *translation = wb_supervisor_field_get_sf_vec3f(translationField);

    // display the robot position
    wb_display_fill_oval(display, width * (translation[X] + GROUND_X / 2) / GROUND_X,
                         height * (-translation[Y] + GROUND_Y / 2) / GROUND_Y, 7, 7);
  }

  wb_robot_cleanup();

  return 0;
}
