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
 * Description:  Example of Sick LMS 291.
 *               The motors position or velocity is set
 *               according to a braitenberg algorithm based
 *               on the values returned by the Sick,
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/display.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32
#define MAX_SPEED 100.0

#define DISPLAY_RECTANGLE_Y_OFFSET 10
#define DISPLAY_WIDTH 200
#define DISPLAY_HEIGHT 110
#define DISPLAY_MAX_RANGE 80
#define DISPLAY_CIRCLE_SIZE 5

#define WHITE 0x00FFFFFF
#define BLACK 0x00000000
#define RED 0x00994444
#define BLUE 0x00444499

// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

// init the background of the display, store it and return its pointer
WbImageRef init_display(WbDeviceTag display) {
  wb_display_fill_rectangle(display, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  wb_display_set_color(display, BLACK);
  wb_display_draw_rectangle(display, (DISPLAY_WIDTH - 2 * DISPLAY_MAX_RANGE) / 2 - 1,
                            (DISPLAY_HEIGHT - DISPLAY_MAX_RANGE) / 2 + DISPLAY_RECTANGLE_Y_OFFSET - 1,
                            2 * DISPLAY_MAX_RANGE + 2, DISPLAY_MAX_RANGE + 2);
  wb_display_draw_text(display, "Sick LMS 291 values", DISPLAY_RECTANGLE_Y_OFFSET, DISPLAY_RECTANGLE_Y_OFFSET);
  wb_display_set_color(display, BLUE);
  wb_display_fill_oval(display, DISPLAY_WIDTH / 2, (DISPLAY_HEIGHT - DISPLAY_MAX_RANGE) / 2 + DISPLAY_RECTANGLE_Y_OFFSET,
                       DISPLAY_CIRCLE_SIZE, DISPLAY_CIRCLE_SIZE);
  WbImageRef background = wb_display_image_copy(display, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  wb_display_set_color(display, RED);
  return background;
}

// display the background and the points of the lidar
void update_display(WbDeviceTag display, WbImageRef background, const float *lidar_values, int lms291_width) {
  wb_display_image_paste(display, background, 0, 0, false);
  int i;
  double angle = -M_PI_2;

  for (i = 0; i < lms291_width; i++) {
    float x = lidar_values[i] * sin(angle);
    float y = lidar_values[i] * cos(angle);
    wb_display_draw_pixel(display, DISPLAY_WIDTH / 2 + x,
                          (DISPLAY_HEIGHT - DISPLAY_MAX_RANGE) / 2 + DISPLAY_RECTANGLE_Y_OFFSET + y);
    angle += M_PI / lms291_width;
  }
}

int main(int argc, char **argv) {
  // init webots stuff
  wb_robot_init();

  // get devices
  WbDeviceTag lms291 = wb_robot_get_device("Sick LMS 291");
  WbDeviceTag direction = wb_robot_get_device("direction");
  WbDeviceTag back_left_wheel = wb_robot_get_device("back_left_wheel");
  WbDeviceTag back_right_wheel = wb_robot_get_device("back_right_wheel");
  WbDeviceTag display = wb_robot_get_device("display");

  // init lms291
  wb_lidar_enable(lms291, TIME_STEP);
  int lms291_width = wb_lidar_get_horizontal_resolution(lms291);
  int max_range = wb_lidar_get_max_range(lms291);

  // init the display
  WbImageRef background = init_display(display);

  // init braitenberg coefficient
  double *braitenberg_coefficients = (double *)malloc(sizeof(double) * lms291_width);
  int i;
  for (i = 0; i < lms291_width; i++) {
    braitenberg_coefficients[i] = gaussian(i, lms291_width / 2, lms291_width / 36);
    // printf("%f, ",braitenberg_coefficients[i]);
  }

  // init motors
  wb_motor_set_position(back_left_wheel, INFINITY);
  wb_motor_set_position(back_right_wheel, INFINITY);

  // perform simulation steps
  while (wb_robot_step(TIME_STEP) != -1) {
    // get lidar values
    const float *lms291_values = wb_lidar_get_range_image(lms291);

    // apply the braitenberg coefficients on the resulted values of the lms291
    double obstacle = 0.0;
    for (i = 0; i < lms291_width; i++) {
      const float value = isinf(lms291_values[i]) ? max_range : lms291_values[i];
      obstacle += braitenberg_coefficients[i] * (1.0 - value / max_range);
    }
    // compute the speed and the direction according to the information about
    // a front obstacle
    double speed = MAX_SPEED * (0.99 - obstacle);
    double dir = obstacle / speed;

    // printf("obstacle=%f dir=%f speed=%f\n",obstacle, dir,speed);

    // update display
    update_display(display, background, lms291_values, lms291_width);

    // set actuators
    wb_motor_set_position(direction, dir);
    wb_motor_set_velocity(back_left_wheel, speed);
    wb_motor_set_velocity(back_right_wheel, speed);
  }

  free(braitenberg_coefficients);
  wb_robot_cleanup();

  return 0;
}
