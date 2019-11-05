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

/*
 * Description:  Example of Hokuyo URG-04LX-UG01.
 *               The velocity of each wheel is set
 *               according to a Braitenberg-like algorithm which takes the values returned by the Hokuyo as input.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32
#define MAX_SPEED 6.4
#define CRUISING_SPEED 5.0
#define OBSTACLE_THRESHOLD 0.3
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9
#define UNUSED_POINT 90

// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

int main(int argc, char **argv) {
  // init webots stuff
  wb_robot_init();

  // get devices
  WbDeviceTag urg04lx = wb_robot_get_device("Hokuyo URG-04LX-UG01");
  WbDeviceTag left_wheel = wb_robot_get_device("wheel_left_joint");
  WbDeviceTag right_wheel = wb_robot_get_device("wheel_right_joint");


  // init urg04lx
  wb_lidar_enable(urg04lx, TIME_STEP);
  wb_lidar_enable_point_cloud(urg04lx);
  const int urg04lx_width = wb_lidar_get_horizontal_resolution(urg04lx);
  const int half_width = urg04lx_width / 2;
  const int max_range = wb_lidar_get_max_range(urg04lx)-2*UNUSED_POINT;
  const double range_threshold = max_range / 20.0;webo
  const float *urg04lx_values = NULL;

  // init braitenberg coefficient
  double *const braitenberg_coefficients = (double *)malloc(sizeof(double) * urg04lx_width);
  int i, j;
  for (i = UNUSED_POINT; i < urg04lx_width-UNUSED_POINT; ++i){
    braitenberg_coefficients[i] = gaussian(i, half_width, (urg04lx_width-2*UNUSED_POINT) / 5);
    printf("b coeff = %lf\n", braitenberg_coefficients[i]);
  }
  // init motors
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);

  // init speed for each wheel
  double left_speed = 0.0, right_speed = 0.0;
  wb_motor_set_velocity(left_wheel, left_speed);
  wb_motor_set_velocity(right_wheel, right_speed);

  // init dynamic variables
  double left_obstacle = 0.0, right_obstacle = 0.0;

  // control loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // get lidar values
    urg04lx_values = wb_lidar_get_range_image(urg04lx);
    // apply the braitenberg coefficients on the resulted values of the urg04lx
    // near obstacle sensed on the left side
    for (i = UNUSED_POINT; i < half_width; ++i) {
      if (urg04lx_values[i] < range_threshold)  // far obstacles are ignored
        left_obstacle += braitenberg_coefficients[i] * (1.0 - urg04lx_values[i] / max_range);
      // near obstacle sensed on the right side
      j = urg04lx_width-UNUSED_POINT - i - 1;
      if (urg04lx_values[j] < range_threshold)
        right_obstacle += braitenberg_coefficients[i] * (1.0 - urg04lx_values[j] / max_range);
    }
    // overall front obstacle
    const double obstacle = left_obstacle + right_obstacle;
    printf("Left = %lf\n", left_obstacle);
    printf("front= %lf\n", obstacle);
    printf("right= %lf\n", right_obstacle);
    // compute the speed according to the information on
    // obstacles
    if (obstacle > OBSTACLE_THRESHOLD) {
      const double speed_factor = (1.0 - DECREASE_FACTOR * obstacle) * MAX_SPEED / obstacle;
      left_speed = speed_factor * left_obstacle;
      right_speed = speed_factor * right_obstacle;
    } else {
      left_speed = CRUISING_SPEED;
      right_speed = CRUISING_SPEED;
    }
    // set actuators
    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);

    // reset dynamic variables to zero
    left_obstacle = 0.0;
    right_obstacle = 0.0;
  }

  free(braitenberg_coefficients);
  wb_robot_cleanup();

  return 0;
}
