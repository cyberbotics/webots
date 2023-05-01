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
 *              by the Lidar as input.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SPEED 4.0
#define MAX_SPEED 6.4
#define CRUISING_SPEED 3.0
#define FAR_OBSTACLE_THRESHOLD 0.2
#define NEAR_OBSTACLE_THRESHOLD 0.6
#define FAST_DECREASE_FACTOR 0.9
#define SLOW_DECREASE_FACTOR 0.5
#define USED_POINTS 545
#define N_SECTOR 5

static WbDeviceTag wheels[4];

// Utility functions to drove the omnidirectional wheels.

static void base_set_wheel_speeds_helper(double speeds[4]) {
  for (int i = 0; i < 4; i++)
    wb_motor_set_velocity(wheels[i], speeds[i]);
}

static void base_forwards() {
  static double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

/*
static void base_backwards() {
  static double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

static void base_strafe_right() {
  static double speeds[4] = {-SPEED, SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

static void base_strafe_left() {
  static double speeds[4] = {SPEED, -SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}


static void base_turn_left() {
  static double speeds[4] = {SPEED, -SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

static void base_turn_right() {
  static double speeds[4] = {-SPEED, SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}
*/

static double check_speed(double speed) {
  if (speed > MAX_SPEED)
    speed = MAX_SPEED;
  return speed;
}

int main(int argc, char **argv) {
  // init webots stuff
  wb_robot_init();

  const int time_step = wb_robot_get_basic_time_step();

  // init motors
  wheels[0] = wb_robot_get_device("wheel_front_left_joint");
  wheels[1] = wb_robot_get_device("wheel_front_right_joint");
  wheels[2] = wb_robot_get_device("wheel_rear_left_joint");
  wheels[3] = wb_robot_get_device("wheel_rear_right_joint");
  for (int i = 0; i < 4; ++i)
    wb_motor_set_position(wheels[i], INFINITY);

  // init lidar
  WbDeviceTag lidar = wb_robot_get_device("base_front_laser");
  wb_lidar_enable(lidar, time_step);
  const int lidar_width = wb_lidar_get_horizontal_resolution(lidar);  // 818 points
  const int front_points = lidar_width * 2 / 3;                       // 545 points
  if (USED_POINTS > front_points)
    fprintf(stderr, "Too many lidar points are used for navigation.\n");
  const int start_point = lidar_width / 3 + (front_points - USED_POINTS) / 2;
  const int end_point = start_point + USED_POINTS;

  // defines sector range
  int sector_range[N_SECTOR] = {0};
  int sector_size = (USED_POINTS - 1.0) / N_SECTOR;
  for (int i = 0; i < N_SECTOR; i++)
    sector_range[i] = (i + 1) * sector_size + start_point;

  // defines useful points (above 2.5m points not used)
  const double range_threshold = 2.5;
  const float *lidar_values = NULL;

  // init dynamic variables
  double left_obstacle = 0.0, right_obstacle = 0.0;
  double front_obstacle = 0.0, front_left_obstacle = 0.0, front_right_obstacle = 0.0;

  base_forwards();
  // control loop
  double left_speed, right_speed;
  while (wb_robot_step(time_step) != -1) {
    // get lidar values
    lidar_values = wb_lidar_get_range_image(lidar);

    for (int i = start_point; i <= end_point; ++i) {
      if (lidar_values[i] < range_threshold) {
        const double v = 1.0 - lidar_values[i] / range_threshold;
        if (i < sector_range[0])
          right_obstacle += v;
        else if (sector_range[0] <= i && i < sector_range[1])
          front_right_obstacle += v;
        else if (sector_range[1] <= i && i < sector_range[2])
          front_obstacle += v;
        else if (sector_range[2] <= i && i < sector_range[3])
          front_left_obstacle += v;
        else if (sector_range[3] <= i && i < sector_range[4] + 1)
          left_obstacle += v;
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
        right_speed = check_speed(speed_factor * front_obstacle);
        left_speed = -right_speed;
      } else {
        left_speed = check_speed(speed_factor * front_obstacle);
        right_speed = -left_speed;
      }
      wb_robot_step(1);

    } else {
      left_speed = CRUISING_SPEED;
      right_speed = CRUISING_SPEED;
    }

    if (!isnan(left_speed) && !isnan(right_speed)) {
      // set actuators
      double speed[4] = {right_speed, left_speed, left_speed, right_speed};
      base_set_wheel_speeds_helper(speed);
    }

    // reset dynamic variables to zero
    left_obstacle = 0.0;
    front_left_obstacle = 0.0;
    front_obstacle = 0.0;
    front_right_obstacle = 0.0;
    right_obstacle = 0.0;
  }

  wb_robot_cleanup();

  return 0;
}
