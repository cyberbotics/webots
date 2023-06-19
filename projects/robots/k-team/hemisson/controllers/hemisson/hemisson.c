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
 * Description:  The hemisson cross-compilation example.
 */

/*
 * This program can be compiled for Webots (with GCC or Visual C++) or
 * for the Hemisson robot with the HemiOS from http://www.hemisson.com
 * using CCS C compiler from http://www.ccsinfo.com
 */

#include "hemisson.h"

#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 64

int main() {
  WbDeviceTag ds0, ds1, ds2;           /* pointers to the distance sensors  */
  WbDeviceTag left_motor, right_motor; /* pointers to the motors  */
  int backward_counter = 0;            /* used to make a long backwards movement */
  double left_speed, right_speed;

  wb_robot_init(); /* Hemisson Initialisation */

  /*
   * It is necessary to use the HEMISSON_* constants as device names to
   * be able to cross-compile for the real Hemisson robot.
   */
  ds0 = wb_robot_get_device(HEMISSON_DS0); /* front right */
  ds1 = wb_robot_get_device(HEMISSON_DS1); /* front left */
  ds2 = wb_robot_get_device(HEMISSON_DS2); /* front */
  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);
  wb_distance_sensor_enable(ds2, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device(HEMISSON_MOTOR_LEFT);
  right_motor = wb_robot_get_device(HEMISSON_MOTOR_RIGHT);
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Using only three sensors we decide wich movement we will have to do.
     * The order of the different tests is very important for the robot not
     * to get stuck against a wall.
     */
    const int ds0_value = wb_distance_sensor_get_value(ds0);
    const int ds1_value = wb_distance_sensor_get_value(ds1);
    const int ds2_value = wb_distance_sensor_get_value(ds2);

    if (backward_counter != 0) {
      backward_counter--;
      left_speed = -1.378;
      right_speed = -0.41;
    } else if (ds2_value > 10) {
      left_speed = -1.378;
      right_speed = 1.378;
    } else if (ds0_value > 10 && ds1_value > 10) {
      backward_counter = 15;
      left_speed = -1.378;
      right_speed = -0.41;
    } else if (ds0_value > 10) {
      left_speed = -1.378;
      right_speed = 1.378;
    } else if (ds1_value > 10) {
      left_speed = 1.378;
      right_speed = -1.378;
    } else {
      left_speed = 1.378;
      right_speed = 1.378;
    }
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }
  wb_robot_cleanup();
  return 0;
}
