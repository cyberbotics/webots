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

// Description:   Demo/Test for the Compass device
//                A robot is equipped with a needle that constantly point towards
//                the north while the moves around.

#include <math.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 8

int main() {
  wb_robot_init();  // necessary to initialize webots stuff

  // get devices
  WbDeviceTag arrow = wb_robot_get_device("arrow");
  WbDeviceTag compass = wb_robot_get_device("compass");
  WbDeviceTag us0 = wb_robot_get_device("us0");
  WbDeviceTag us1 = wb_robot_get_device("us1");

  // enable the devices
  wb_compass_enable(compass, TIME_STEP);
  wb_distance_sensor_enable(us0, TIME_STEP);
  wb_distance_sensor_enable(us1, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // run simulation
  while (wb_robot_step(TIME_STEP) != -1) {
    // read distance sensors
    double d0 = wb_distance_sensor_get_value(us0);
    double d1 = wb_distance_sensor_get_value(us1);
    if (d0 < 100 || d1 < 100) {
      // in case of collision turn left
      wb_motor_set_velocity(left_motor, -5);
      wb_motor_set_velocity(right_motor, 5);
    } else {
      // otherwise go straight
      wb_motor_set_velocity(left_motor, 5);
      wb_motor_set_velocity(right_motor, 5);
    }

    // read compass and rotate arrow accordingly
    const double *north = wb_compass_get_values(compass);
    double angle = atan2(north[1], north[0]);
    wb_motor_set_position(arrow, angle);
  }

  wb_robot_cleanup();

  return 0;  // never reached
}
