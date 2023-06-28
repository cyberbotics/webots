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

#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Macros to determine min or max between 2 inputs.
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

static int timestep;

static void step() {
  if (wb_robot_step(timestep) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

int main(int argc, char **argv) {
  wb_robot_init();

  timestep = (int)wb_robot_get_basic_time_step();

  // Get and enable motors and sensors.
  WbDeviceTag left_motor = wb_robot_get_device("left motor");
  WbDeviceTag right_motor = wb_robot_get_device("right motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  WbDeviceTag right_motor_sensor = wb_robot_get_device("right motor sensor");
  wb_position_sensor_enable(right_motor_sensor, timestep);
  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, timestep);

  // Set LED colors.
  wb_led_set(wb_robot_get_device("left motor led"), 0xFF0000);
  wb_led_set(wb_robot_get_device("right motor led"), 0x00FF00);
  wb_led_set(wb_robot_get_device("distance sensor led"), 0x0000FF);

  const double target_position = 15.5;
  const double max_speed = 10.0;
  bool forward = false;

  // In loop; wait until an object is detected, then move the robot forward and then backward.
  while (true) {
    step();

    double delta;
    if (forward) {
      const double position = wb_position_sensor_get_value(right_motor_sensor);
      delta = MIN(max_speed, 2.0 * (target_position - position));
      wb_motor_set_velocity(left_motor, -delta);
      wb_motor_set_velocity(right_motor, delta);
    } else {
      const double position = wb_position_sensor_get_value(right_motor_sensor);
      delta = MIN(max_speed, 2.0 * position);
      wb_motor_set_velocity(left_motor, delta);
      wb_motor_set_velocity(right_motor, -delta);
    }

    if (fabs(delta) < 0.05) {
      forward = !forward;
      if (forward)
        while (wb_distance_sensor_get_value(distance_sensor) > 120)
          step();
    }
  }

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
