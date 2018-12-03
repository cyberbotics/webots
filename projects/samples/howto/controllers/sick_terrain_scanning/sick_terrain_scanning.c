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
#include <webots/emitter.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 64
#define MAX_SPEED 6.4
#define CRUISING_SPEED 5.0
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9

// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

int main(int argc, char **argv) {
  // init webots stuff
  wb_robot_init();

  // get devices
  WbDeviceTag lms291 = wb_robot_get_device("Sick LMS 291");
  WbDeviceTag front_left_wheel = wb_robot_get_device("front left wheel");
  WbDeviceTag front_right_wheel = wb_robot_get_device("front right wheel");
  WbDeviceTag back_left_wheel = wb_robot_get_device("back left wheel");
  WbDeviceTag back_right_wheel = wb_robot_get_device("back right wheel");

  // init lms291
  wb_lidar_enable(lms291, TIME_STEP);
  int lms291_width = wb_lidar_get_horizontal_resolution(lms291);
  int half_width = lms291_width / 2;
  int max_range = wb_lidar_get_max_range(lms291);
  double range_threshold = max_range / 20.0;

  // init braitenberg coefficient
  double *braitenberg_coefficients = (double *)malloc(sizeof(double) * lms291_width);
  int i, j;
  for (i = 0; i < lms291_width; i++)
    braitenberg_coefficients[i] = gaussian(i, lms291_width / 2, lms291_width / 5);

  // init motors
  wb_motor_set_position(front_left_wheel, INFINITY);
  wb_motor_set_position(front_right_wheel, INFINITY);
  wb_motor_set_position(back_left_wheel, INFINITY);
  wb_motor_set_position(back_right_wheel, INFINITY);

  // init speed for each wheel
  double back_left_speed = 0.0, back_right_speed = 0.0;
  double front_left_speed = 0.0, front_right_speed = 0.0;

  // init dynamic variables
  double left_obstacle = 0.0, right_obstacle = 0.0;
  double speed_factor = 1.0;

  // Communications
  WbDeviceTag communication;
  communication = wb_robot_get_device("emitter");

  // control loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // get lidar values
    const float *lms291_values = wb_lidar_get_range_image(lms291);

    // send lidar values to supervisor
    wb_emitter_send(communication, lms291_values, 180 * sizeof(float));

    // apply the braitenberg coefficients on the resulted values of the lms291
    // near obstacle sensed on the left side
    for (i = 0; i < half_width; i++) {
      if (lms291_values[i] < range_threshold)  // far obstacles are ignored
        left_obstacle += braitenberg_coefficients[i] * (1.0 - lms291_values[i] / max_range);
      // near obstacle sensed on the right side
      j = lms291_width - i - 1;
      if (lms291_values[j] < range_threshold)
        right_obstacle += braitenberg_coefficients[i] * (1.0 - lms291_values[j] / max_range);
    }
    // overall front obstacle
    const double obstacle = left_obstacle + right_obstacle;
    // compute the speed according to the information on
    // obstacles
    if (obstacle > OBSTACLE_THRESHOLD) {
      speed_factor = (1.0 - DECREASE_FACTOR * obstacle) * MAX_SPEED / obstacle;
      front_left_speed = speed_factor * left_obstacle;
      front_right_speed = speed_factor * right_obstacle;
      back_left_speed = BACK_SLOWDOWN * front_left_speed;
      back_right_speed = BACK_SLOWDOWN * front_right_speed;
    } else {
      back_left_speed = CRUISING_SPEED;
      back_right_speed = CRUISING_SPEED;
      front_left_speed = CRUISING_SPEED;
      front_right_speed = CRUISING_SPEED;
    }
    // set actuators
    wb_motor_set_velocity(front_left_wheel, front_left_speed);
    wb_motor_set_velocity(front_right_wheel, front_right_speed);
    wb_motor_set_velocity(back_left_wheel, back_left_speed);
    wb_motor_set_velocity(back_right_wheel, back_right_speed);

    // reset dynamic variables to zero
    left_obstacle = 0.0;
    right_obstacle = 0.0;
  }

  free(braitenberg_coefficients);
  wb_robot_cleanup();

  return 0;
}
