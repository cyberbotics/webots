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
 * Description:  This robot moves randomly in a maze
 *               according to its distance sensors.
 *               It has 2 Displays devices, one is used
 *               to display its emotion (randomly chosen)
 *               on the screen located on its top, and the
 *               other one is used to display the camera
 *               image with extra information (yellow blob)
 */

#include <webots/camera.h>
#include <webots/display.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define SPEED 4
#define MAX_SPEED 10
#define TIME_STEP 64

#define EMOTICON_WIDTH 14
#define EMOTICON_HEIGHT 14
#define EMOTICONS_NUMBER_X 5
#define EMOTICONS_NUMBER_Y 11

// main function
int main() {
  // init Webots controller
  wb_robot_init();

  // First we get a handler to the devices
  WbDeviceTag ds0 = wb_robot_get_device("ds0");
  WbDeviceTag ds1 = wb_robot_get_device("ds1");
  WbDeviceTag camera = wb_robot_get_device("camera");
  WbDeviceTag emoticon_display = wb_robot_get_device("emoticon_display");
  WbDeviceTag camera_display = wb_robot_get_device("camera_display");

  // enable them
  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);
  wb_camera_enable(camera, TIME_STEP);

  // get a handler to the motors and set target position to infinity (speed control).
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // attach the camera with the display
  int width = wb_display_get_width(camera_display);
  assert(width == wb_camera_get_width(camera));
  int height = wb_display_get_height(camera_display);
  assert(height == wb_camera_get_height(camera));
  wb_display_attach_camera(camera_display, camera);

  // import png image containing the emoticons
  WbImageRef emoticonsImage = wb_display_image_load(emoticon_display, "emoticons.png");

  // Set the yellow color and the font once for all
  // to draw the overlay information
  wb_display_set_color(camera_display, 0xFFFF00);
  wb_display_set_font(camera_display, "Palatino Linotype", 16, true);

  int counter = 0;      // "time steps counter"
  int nop_counter = 0;  // "no operation counter": if positive the robot won't actuate the motors until the counter reaches 0.

  // initialize random seed:
  srand(time(NULL));

  while (wb_robot_step(TIME_STEP) != -1) {
    // increment the counter
    counter++;

    // get the sensors values
    double ds0_value = wb_distance_sensor_get_value(ds0);
    double ds1_value = wb_distance_sensor_get_value(ds1);
    const unsigned char *image = wb_camera_get_image(camera);

    // modify the emoticon every 30 steps
    if (counter % 30 == 1) {
      int x = -EMOTICON_WIDTH * (rand() % EMOTICONS_NUMBER_X);
      int y = -EMOTICON_HEIGHT * (rand() % EMOTICONS_NUMBER_Y);
      wb_display_image_paste(emoticon_display, emoticonsImage, x, y, true);
    }

    // display a rectangle around the yellow boxes
    // 1. clear the display
    wb_display_set_alpha(camera_display, 0.0);
    wb_display_fill_rectangle(camera_display, 0, 0, width, height);
    wb_display_set_alpha(camera_display, 1.0);
    // 2. detect the yellow blob
    int minX = width;
    int minY = height;
    int maxX = 0;
    int maxY = 0;
    int x, y;
    for (y = 0; y < height; ++y) {
      for (x = 0; x < width; ++x) {
        int r = wb_camera_image_get_red(image, width, x, y);
        int g = wb_camera_image_get_green(image, width, x, y);
        int b = wb_camera_image_get_blue(image, width, x, y);
        bool is_yellow = r > 80 && g > 80 && b < 40 && abs(r - g) < 5;
        if (is_yellow) {
          if (x < minX)
            minX = x;
          if (y < minY)
            minY = y;
          if (x > maxX)
            maxX = x;
          if (y > maxY)
            maxY = y;
        }
      }
    }
    // draw the blob on the display if visible
    if (minX < maxX && minY < maxY) {
      wb_display_draw_rectangle(camera_display, minX, minY, maxX - minX, maxY - minY);
      wb_display_draw_text(camera_display, "Yellow blob", minX, minY - 20);
    }

    // compute motor's speed with a collision avoidance
    // algorithm having a random factor for a better exploration
    double left_speed, right_speed;
    if (nop_counter > 0) {
      nop_counter--;
      continue;
    } else if (ds0_value > 100.0 && ds1_value > 100.0) {  // front obstacle
      left_speed = -SPEED;
      right_speed = SPEED;
      nop_counter = rand() % 20;  // give enough time to turn.
    } else {
      left_speed = SPEED + (ds0_value - ds1_value) / 7.0 + (rand() % SPEED - SPEED / 2.0);
      left_speed = (left_speed <= MAX_SPEED) ? left_speed : MAX_SPEED;
      left_speed = (left_speed >= -MAX_SPEED) ? left_speed : -MAX_SPEED;
      right_speed = SPEED - (ds0_value - ds1_value) / 11.0 + (rand() % SPEED - SPEED / 2.0);
      right_speed = (right_speed <= MAX_SPEED) ? right_speed : MAX_SPEED;
      right_speed = (right_speed >= -MAX_SPEED) ? right_speed : -MAX_SPEED;
    }

    // set the actuators values
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_display_image_delete(emoticon_display, emoticonsImage);

  wb_robot_cleanup();

  return 0;
}
