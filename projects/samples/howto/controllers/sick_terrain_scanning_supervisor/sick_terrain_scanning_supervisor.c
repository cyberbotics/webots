/*
 * Copyright 1996-2018 Cyberbotics Ltd.
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
 * Description:  Example of Sick LMS 291 Terrain Scannings with obstacle avoidance.
 * Thanks to Angelos Amanatiadis (aamanat@ee.duth.gr)
 */

#include <math.h>
#include <stdlib.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 64
#define pi 3.14159265358979323846
#define X 0
#define Y 1
#define Z 2

// size of the ground
#define GROUND_X 100
#define GROUND_Z 100

static WbDeviceTag display, display2;
static WbImageRef background, background2;
static int width, height, width2, height2;
static int px[180];

// Display SICK laser data
static void update_display(float *scanvalue, int laser_width) {
  int half_width2 = width2 / 2;
  int half_height2 = height2 / 2;
  // (re)paint background
  wb_display_image_paste(display2, background2, 0, 0, false);
  // draw polygon scannings
  int py[laser_width];
  int i;
  for (i = 0; i < laser_width; i++) {
    if (scanvalue[i] > 25)  // range up to 25 metres
      scanvalue[i] = 25;
    px[i] = half_width2 - (cos(i * pi / 180)) * ((90 * scanvalue[i]) / 25);
    py[i] = half_height2 - (sin(i * pi / 180)) * ((90 * scanvalue[i]) / 25);
  }
  // update Scan Display
  wb_display_draw_polygon(display2, px, py, 180);
}

// main function
int main() {
  // init Webtos stuff
  wb_robot_init();

  // First we get a handler to devices
  display = wb_robot_get_device("ground_display");
  display2 = wb_robot_get_device("display2");

  // Second we get a handler from devices outsise supervisor controller
  WbNodeRef mybot = wb_supervisor_node_get_from_def("PIONEER_3AT");
  WbNodeRef mysick = wb_supervisor_node_get_from_def("SICK");

  // get the properties of the Ground Layer Display
  width = wb_display_get_width(display);
  height = wb_display_get_height(display);

  // get and set the properties of the Scan Display
  width2 = wb_display_get_width(display2);
  height2 = wb_display_get_height(display2);
  wb_display_set_color(display2, 0x0000ff);

  int pxfill[180];
  int pyfill[180];

  // Communications
  WbDeviceTag communication;
  communication = wb_robot_get_device("receiver");
  wb_receiver_enable(communication, TIME_STEP);

  // prepare stuff to get the
  // ROBOT(IROBOT_CREATE).translation & rotation field, and SICK configuration
  WbFieldRef translationField = wb_supervisor_node_get_field(mybot, "translation");
  WbFieldRef rotationField = wb_supervisor_node_get_field(mybot, "rotation");
  WbFieldRef widthsick = wb_supervisor_node_get_field(mysick, "resolution");
  const double *translation;
  const double *rotation;
  int laser_width;

  // set the background (otherwise an empty ground is displayed at this step)
  background = wb_display_image_load(display, "dirty.png");
  wb_display_image_paste(display, background, 0, 0, false);
  background2 = wb_display_image_load(display2, "grid_display.png");
  wb_display_image_paste(display2, background2, 0, 0, false);
  wb_display_set_alpha(display, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    // get the SICK data from pioneer through communications
    int queueLength = wb_receiver_get_queue_length(communication);
    if (queueLength <= 0)
      continue;

    float *buffer = (float *)wb_receiver_get_data(communication);

    // Update the translation & rotation field
    laser_width = wb_supervisor_field_get_sf_int32(widthsick);
    translation = wb_supervisor_field_get_sf_vec3f(translationField);
    rotation = wb_supervisor_field_get_sf_rotation(rotationField);

    // display the robot position
    int i;
    for (i = 0; i < laser_width; i++) {
      if (buffer[i] > 25)  // range up to 25 metres
        buffer[i] = 25;
      pxfill[i] = width * (translation[X] + GROUND_X / 2 + sin(rotation[3] - pi / 2 - i * pi / 180) * buffer[i]) / GROUND_X;
      pyfill[i] = height * (translation[Z] + GROUND_Z / 2 + cos(rotation[3] - pi / 2 - i * pi / 180) * buffer[i]) / GROUND_Z;
    }

    wb_display_fill_polygon(display, pxfill, pyfill, 180);

    // display SICK laser data
    update_display(buffer, laser_width);

    // fetch new laser data through communication
    wb_receiver_next_packet(communication);
  }

  wb_robot_cleanup();

  return 0;
}
