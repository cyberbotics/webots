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

#include "sc_control.h"

#include <webots/camera.h>
#include <webots/display.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define LEFT 0
#define RIGHT 1
#define HALF_CAMERA_DISTANCE 0.04

static WbDeviceTag cameras[2] = {0, 0};
static WbDeviceTag motors[2] = {0, 0};
static WbDeviceTag display = 0;
static WbDeviceTag distance_sensor;

static int width = 0;
static int height = 0;
static int fade_out = 0;
static double focus = 0.0;

static bool show = true;

static unsigned char *im = NULL;

bool sc_init(int time_step, bool show_on_display, int fade_out_size, double fixed_focus) {
  show = show_on_display;
  fade_out = fade_out_size;
  focus = fixed_focus;

  cameras[LEFT] = wb_robot_get_device("sc_left_camera");
  cameras[RIGHT] = wb_robot_get_device("sc_right_camera");
  motors[LEFT] = wb_robot_get_device("sc_left_motor");
  motors[RIGHT] = wb_robot_get_device("sc_right_motor");
  if (focus <= 0.0) {
    distance_sensor = wb_robot_get_device("sc_distance_sensor");
    wb_distance_sensor_enable(distance_sensor, time_step);
  }
  if (show)
    display = wb_robot_get_device("sc_display");

  wb_camera_enable(cameras[LEFT], time_step);
  wb_camera_enable(cameras[RIGHT], time_step);

  width = wb_camera_get_width(cameras[LEFT]);
  height = wb_camera_get_height(cameras[LEFT]);

  if (width != wb_camera_get_width(cameras[RIGHT]) || height != wb_camera_get_height(cameras[RIGHT])) {
    fprintf(stderr, "Dimensions of the cameras doesn't match\n");
    return false;
  }
  if (show) {
    if (width != wb_display_get_width(display) || height != wb_display_get_height(display)) {
      fprintf(stderr, "Dimensions of the display doesn't match\n");
      return false;
    }
  }

  im = (unsigned char *)malloc(4 * width * height);

  return (im != NULL);
}

void sc_update() {
  const unsigned char *left_im = wb_camera_get_image(cameras[LEFT]);
  const unsigned char *right_im = wb_camera_get_image(cameras[RIGHT]);
  if (left_im == NULL || right_im == NULL)
    return;

  double d = focus;
  if (focus <= 0.0)
    d = wb_distance_sensor_get_value(distance_sensor);

  double alpha = M_PI_2 - atan(d / HALF_CAMERA_DISTANCE);
  wb_motor_set_position(motors[LEFT], -alpha);
  wb_motor_set_position(motors[RIGHT], alpha);

  int i, j, index;
  for (j = 0; j < height; j++) {
    for (i = 0; i < width; i++) {
      index = 4 * (j * width + i);
      im[index] = right_im[index];
      im[index + 1] = right_im[index + 1];
      im[index + 2] = left_im[index + 2];
      im[index + 3] = 255;
    }
  }
  if (fade_out > 0) {
    for (j = 0; j < height; j++) {
      for (i = 0; i < fade_out; i++) {
        index = 4 * (j * width + i);
        im[index] = im[index] * i / fade_out;
        im[index + 1] = im[index + 1] * i / fade_out;
        im[index + 2] = im[index + 2] * i / fade_out;
        index = 4 * (j * width + (width - 1 - i));
        im[index] = im[index] * i / fade_out;
        im[index + 1] = im[index + 1] * i / fade_out;
        im[index + 2] = im[index + 2] * i / fade_out;
      }
    }
  }

  if (show) {
    WbImageRef ir = wb_display_image_new(display, width, height, im, WB_IMAGE_BGRA);
    wb_display_image_paste(display, ir, 0, 0, false);
    wb_display_image_delete(display, ir);
  }
}

void sc_cleanup() {
  if (im)
    free(im);
}

const unsigned char *sc_get_image() {
  return im;
}
