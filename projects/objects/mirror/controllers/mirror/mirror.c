/*
 * Copyright 1996-2019 Cyberbotics Ltd.
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

#include <webots/camera.h>
#include <webots/display.h>
#include <webots/robot.h>

#include <stdio.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();

  // get and activate the camera
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  // get display and attach the camera to it
  WbDeviceTag display = wb_robot_get_device("display");
  wb_display_attach_camera(display, camera);

  wb_robot_step(5 * TIME_STEP);

  // check the red color
  const int width = wb_camera_get_width(camera);
  const unsigned char *image = wb_camera_get_image(camera);
  int x = 30;
  int y = 350;
  int r = wb_camera_image_get_red(image, width, x, y);
  int g = wb_camera_image_get_green(image, width, x, y);
  int b = wb_camera_image_get_blue(image, width, x, y);
  char data[128];
  sprintf(data, "%d %d %d", r, g, b);
  printf("data %s\n", data);
  wb_robot_set_custom_data(data);

  // empty main loop
  while (wb_robot_step(TIME_STEP) != -1) {
  }

  wb_robot_cleanup();

  return 0;
}
