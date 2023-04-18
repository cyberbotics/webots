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
 * Description:  A Braitenberg-like controller moving a Pioneer 3-DX equipped with a kinect.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>

#define MAX_SPEED 5.24
#define CRUISING_SPEED 5
#define TOLERANCE -0.1
#define OBSTACLE_THRESHOLD 0.5
#define SLOWDOWN_FACTOR 0.5

static WbDeviceTag left_wheel, right_wheel;
static WbDeviceTag kinectColor;
static WbDeviceTag kinectRange;
const float *kinect_values;
static int time_step = 0;

int main() {
  // necessary to initialize Webots
  wb_robot_init();
  // get time step and robot's devices
  time_step = wb_robot_get_basic_time_step();
  left_wheel = wb_robot_get_device("left wheel");
  right_wheel = wb_robot_get_device("right wheel");
  kinectColor = wb_robot_get_device("kinect color");
  kinectRange = wb_robot_get_device("kinect range");
  wb_camera_enable(kinectColor, time_step);
  wb_range_finder_enable(kinectRange, time_step);
  const int kinect_width = wb_range_finder_get_width(kinectRange);
  const int kinect_height = wb_range_finder_get_height(kinectRange);
  const int half_width = kinect_width / 2;
  const int view_height = kinect_height / 2 + 10;
  const double max_range = wb_range_finder_get_max_range(kinectRange);
  const double range_threshold = 1.5;
  const double inv_max_range_times_width = 1.0 / (max_range * kinect_width);
  // set motors' positions
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  // set speeds
  wb_motor_set_velocity(left_wheel, 0.0);
  wb_motor_set_velocity(right_wheel, 0.0);

  // init dynamic variables
  double left_obstacle = 0.0, right_obstacle = 0.0;
  double left_speed, right_speed;
  float value;
  int i;

  // control loop
  while (wb_robot_step(time_step) != -1) {
    // get range-finder values
    kinect_values = (float *)wb_range_finder_get_range_image(kinectRange);

    for (i = 0; i < half_width; ++i) {
      // record near obstacle sensed on the left side
      value = wb_range_finder_image_get_depth(kinect_values, kinect_width, i, view_height);
      if (value < range_threshold)  // far obstacles are ignored
        left_obstacle += value;
      // record near obstacle sensed on the right side
      value = wb_range_finder_image_get_depth(kinect_values, kinect_width, kinect_width - i, view_height);
      if (value < range_threshold)
        right_obstacle += value;
    }

    double obstacle = left_obstacle + right_obstacle;

    // compute the speed according to the information on
    // possible left and right obstacles
    if (obstacle > 0.0) {
      obstacle = 1.0 - obstacle * inv_max_range_times_width;  // compute the relevant overall quantity of obstacle
      const double speed_factor = (obstacle > OBSTACLE_THRESHOLD) ? 0.0 : SLOWDOWN_FACTOR;
      const double delta_obstacle = -(left_obstacle - right_obstacle) * inv_max_range_times_width;
      if (delta_obstacle > TOLERANCE) {
        left_speed = CRUISING_SPEED;
        right_speed = speed_factor * CRUISING_SPEED;
      } else {
        left_speed = speed_factor * CRUISING_SPEED;
        right_speed = CRUISING_SPEED;
      }
    } else {
      left_speed = CRUISING_SPEED;
      right_speed = CRUISING_SPEED;
    }

    // set speeds
    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);

    // run simulation
    wb_robot_step(time_step);
    left_obstacle = 0.0;
    right_obstacle = 0.0;
  }

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
