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

#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
#define BASE_SPEED 1.5

// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

int main(int argc, char **argv) {
  wb_robot_init();

  // get and enable the lidar
  WbDeviceTag lidar = wb_robot_get_device("LDS-01");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);

  // get lidar motor and enable rotation (only for visualization, no effect on the sensor)
  WbDeviceTag lidar_main_motor = wb_robot_get_device("LDS-01_main_motor");
  WbDeviceTag lidar_secondary_motor = wb_robot_get_device("LDS-01_secondary_motor");
  wb_motor_set_position(lidar_main_motor, INFINITY);
  wb_motor_set_position(lidar_secondary_motor, INFINITY);
  wb_motor_set_velocity(lidar_main_motor, 30.0);
  wb_motor_set_velocity(lidar_secondary_motor, 60.0);

  // get the motors and enable velocity control
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_motor_set_velocity(left_motor, 0.0);

  const int lidar_width = wb_lidar_get_horizontal_resolution(lidar);
  const double lidar_max_range = wb_lidar_get_max_range(lidar);

  // init braitenberg coefficient
  double *braitenberg_coefficients = (double *)malloc(sizeof(double) * lidar_width);
  int i;
  for (i = 0; i < lidar_width; i++)
    braitenberg_coefficients[i] = 6 * gaussian(i, lidar_width / 4, lidar_width / 12);

  while (wb_robot_step(TIME_STEP) != -1) {
    double left_speed = BASE_SPEED, right_speed = BASE_SPEED;

    // get lidar values
    const float *lidar_values = wb_lidar_get_range_image(lidar);

    // apply the braitenberg coefficients on the resulted values of the lidar
    for (i = 0.25 * lidar_width; i < 0.5 * lidar_width; i++) {
      const int j = lidar_width - i - 1;
      const int k = i - 0.25 * lidar_width;
      if (lidar_values[i] != INFINITY && !isnan(lidar_values[i]) && lidar_values[j] != INFINITY && !isnan(lidar_values[j])) {
        left_speed +=
          braitenberg_coefficients[k] * ((1.0 - lidar_values[i] / lidar_max_range) - (1.0 - lidar_values[j] / lidar_max_range));
        right_speed +=
          braitenberg_coefficients[k] * ((1.0 - lidar_values[j] / lidar_max_range) - (1.0 - lidar_values[i] / lidar_max_range));
      }
    }

    // apply computed velocities
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  };

  free(braitenberg_coefficients);
  wb_robot_cleanup();

  return 0;
}
