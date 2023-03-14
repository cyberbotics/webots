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
 * Description: Simple avoidance controller
 *              The velocity of each wheel is set according to a
 *              Braitenberg-like algorithm which takes the values returned
 *              by the Hokuyo URG-04LX-UG01 as input.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAX_SPEED 6.4
#define CRUISING_SPEED 3.0
#define FAR_OBSTACLE_THRESHOLD 0.4
#define NEAR_OBSTACLE_THRESHOLD 0.7
#define FAST_DECREASE_FACTOR 0.9
#define SLOW_DECREASE_FACTOR 0.5
#define UNUSED_POINT 83
#define N_SECTOR 5

double check_speed(double speed) {
  if (speed > MAX_SPEED)
    speed = MAX_SPEED;
  return speed;
}

int main(int argc, char **argv) {
  // init webots stuff
  wb_robot_init();

  const int time_step = wb_robot_get_basic_time_step();

  // get devices
  WbDeviceTag urg04lx = wb_robot_get_device("Hokuyo URG-04LX-UG01");
  WbDeviceTag left_wheel = wb_robot_get_device("wheel_left_joint");
  WbDeviceTag right_wheel = wb_robot_get_device("wheel_right_joint");

  // init urg04lx
  wb_lidar_enable(urg04lx, time_step);
  const int urg04lx_width = wb_lidar_get_horizontal_resolution(urg04lx);  // 667 points

  // defines sector range
  int sector_range[N_SECTOR] = {0};
  int sector_size = (urg04lx_width - 2.0 * UNUSED_POINT - 1.0) / N_SECTOR;
  for (int i = 0; i < N_SECTOR; i++)
    sector_range[i] = UNUSED_POINT + (i + 1) * sector_size;

  // defines usefull points (above 5.6m / 2.0 = 2.8m, points not used)
  const float max_range = wb_lidar_get_max_range(urg04lx);
  const double range_threshold = max_range / 2.0;
  const float *urg04lx_values = NULL;

  // init motors
  double left_speed = 0.0, right_speed = 0.0;
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, left_speed);
  wb_motor_set_velocity(right_wheel, right_speed);

  // init dynamic variables
  double left_obstacle = 0.0, right_obstacle = 0.0;
  double front_obstacle = 0.0, front_left_obstacle = 0.0, front_right_obstacle = 0.0;

  // control loop
  while (wb_robot_step(time_step) != -1) {
    // get lidar values
    urg04lx_values = wb_lidar_get_range_image(urg04lx);

    for (int i = UNUSED_POINT; i < urg04lx_width - UNUSED_POINT - 1; ++i) {
      if (urg04lx_values[i] < range_threshold) {
        if (i < sector_range[0])  // [83-182]
          left_obstacle += (1.0 - urg04lx_values[i] / max_range);
        if (sector_range[0] <= i && i < sector_range[1])  // [183-282]
          front_left_obstacle += (1.0 - urg04lx_values[i] / max_range);
        if (sector_range[1] <= i && i < sector_range[2])  // [283-382]
          front_obstacle += (1.0 - urg04lx_values[i] / max_range);
        if (sector_range[2] <= i && i < sector_range[3])  // [383-482]
          front_right_obstacle += (1.0 - urg04lx_values[i] / max_range);
        if (sector_range[3] <= i && i < sector_range[4] + 1)  // [483-583]
          right_obstacle += (1.0 - urg04lx_values[i] / max_range);
      }
    }
    left_obstacle /= sector_size;
    front_left_obstacle /= sector_size;
    front_obstacle /= sector_size;
    front_right_obstacle /= sector_size;
    right_obstacle /= sector_size;

    // compute the speed according to the information on obstacles
    if (left_obstacle > right_obstacle && left_obstacle > NEAR_OBSTACLE_THRESHOLD) {
      const double speed_factor = (1.0 - FAST_DECREASE_FACTOR * left_obstacle) * MAX_SPEED / left_obstacle;
      left_speed = check_speed(speed_factor * left_obstacle);
      right_speed = check_speed(speed_factor * right_obstacle);

    } else if (front_left_obstacle > front_right_obstacle && front_left_obstacle > FAR_OBSTACLE_THRESHOLD) {
      const double speed_factor = (1.0 - SLOW_DECREASE_FACTOR * front_left_obstacle) * MAX_SPEED / front_left_obstacle;
      left_speed = check_speed(speed_factor * front_left_obstacle);
      right_speed = check_speed(speed_factor * front_right_obstacle);

    } else if (front_right_obstacle > front_left_obstacle && front_right_obstacle > FAR_OBSTACLE_THRESHOLD) {
      const double speed_factor = (1.0 - SLOW_DECREASE_FACTOR * front_right_obstacle) * MAX_SPEED / front_right_obstacle;
      left_speed = check_speed(speed_factor * front_left_obstacle);
      right_speed = check_speed(speed_factor * front_right_obstacle);

    } else if (right_obstacle > left_obstacle && right_obstacle > NEAR_OBSTACLE_THRESHOLD) {
      const double speed_factor = (1.0 - FAST_DECREASE_FACTOR * right_obstacle) * MAX_SPEED / right_obstacle;
      left_speed = check_speed(speed_factor * left_obstacle);
      right_speed = check_speed(speed_factor * right_obstacle);

    } else if (front_obstacle > NEAR_OBSTACLE_THRESHOLD) {
      const double speed_factor = (1.0 - FAST_DECREASE_FACTOR * front_obstacle) * MAX_SPEED / front_obstacle;
      // more obstacles on the right, so make a left u-turn to avoid being stuck
      if (front_right_obstacle > front_left_obstacle || right_obstacle > left_obstacle) {
        left_speed = -check_speed(speed_factor * front_obstacle);
        right_speed = check_speed(speed_factor * front_obstacle);
      } else {
        left_speed = check_speed(speed_factor * front_obstacle);
        right_speed = -check_speed(speed_factor * front_obstacle);
      }
      wb_robot_step(1);

    } else {
      left_speed = CRUISING_SPEED;
      right_speed = CRUISING_SPEED;
    }

    // set actuators
    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);

    // reset dynamic variables to zero
    left_obstacle = 0.0;
    front_left_obstacle = 0.0;
    front_obstacle = 0.0;
    front_right_obstacle = 0.0;
    right_obstacle = 0.0;
  }

  // free(braitenberg_coefficients);
  wb_robot_cleanup();

  return 0;
}
