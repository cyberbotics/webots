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
 * Description:  Simulation of a lidar
 */

#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <stdio.h>

#define TIME_STEP 32
#define LEFT 0
#define RIGHT 1

int main(int argc, char **argv) {
  // iterator used to parse loops
  int i, k;

  // init Webots stuff
  wb_robot_init();

  // init camera
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);

  // init distance sensors
  WbDeviceTag us[2];
  double us_values[2];
  us[LEFT] = wb_robot_get_device("us0");
  us[RIGHT] = wb_robot_get_device("us1");
  for (i = 0; i < 2; ++i)
    wb_distance_sensor_enable(us[i], TIME_STEP);

  // get a handler to the motors and set target position to infinity (speed control).
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // set empirical coefficients for collision avoidance
  double coefficients[2][2] = {{12.0, -6.0}, {-10.0, 8.0}};
  double base_speed = 6.0;

  // init speed values
  double speed[2];

  while (wb_robot_step(TIME_STEP) != -1) {
    // read sensors
    for (i = 0; i < 2; ++i)
      us_values[i] = wb_distance_sensor_get_value(us[i]);

    // compute speed
    for (i = 0; i < 2; ++i) {
      speed[i] = 0.0;
      for (k = 0; k < 2; ++k)
        speed[i] += us_values[k] * coefficients[i][k];
    }

    // set actuators
    wb_motor_set_velocity(left_motor, base_speed + speed[LEFT]);
    wb_motor_set_velocity(right_motor, base_speed + speed[RIGHT]);
  }

  wb_robot_cleanup();

  return 0;
}
