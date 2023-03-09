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
 * Description:  An example of a controller using a light sensor device.
 */

#include <stdio.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAX_SPEED 10
#define TIME_STEP 64

int main() {
  WbDeviceTag ls0, ls1, left_motor, right_motor;

  wb_robot_init();

  /* get a handler to the distance sensors. */
  ls0 = wb_robot_get_device("ls0");
  ls1 = wb_robot_get_device("ls1");

  wb_light_sensor_enable(ls0, TIME_STEP);
  wb_light_sensor_enable(ls1, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  printf("Move the light (shift + drag mouse), the robot should follow it.\n");

  while (wb_robot_step(TIME_STEP) != 1) {
    /* read sensor values */
    const double ls0_value = wb_light_sensor_get_value(ls0);
    const double ls1_value = wb_light_sensor_get_value(ls1);

    double left_speed = (1024 - ls0_value) / 100.0;
    left_speed = (left_speed < MAX_SPEED) ? left_speed : MAX_SPEED;
    double right_speed = (1024 - ls1_value) / 100.0;
    right_speed = (right_speed < MAX_SPEED) ? right_speed : MAX_SPEED;

    /* Set the motor speeds. */
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
