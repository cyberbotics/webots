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
 * Author:        Simon Blanchoud
 */

#include <stdio.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 50
#define OPEN_GRIP 0.029
#define CLOSED_GRIP 0.005

static WbDeviceTag motor, left_grip, right_grip, ds, left_motor, right_motor;

static void set_grip_position(double pos) {
  wb_motor_set_position(left_grip, pos);
  wb_motor_set_position(right_grip, pos);
}

int main() {
  int i = 0;

  wb_robot_init();

  motor = wb_robot_get_device("motor");
  left_grip = wb_robot_get_device("left grip");
  right_grip = wb_robot_get_device("right grip");
  ds = wb_robot_get_device("ds"); /* distance sensor in the gripper */

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  wb_distance_sensor_enable(ds, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != 1) {
    if (i == 0)
      set_grip_position(OPEN_GRIP); /* open the gripper */
    else if (i == 20)
      wb_motor_set_position(motor, 0.0); /* arm down */
    else if (i == 40)
      set_grip_position(CLOSED_GRIP); /* close the gripper */
    else if (i == 80)
      wb_motor_set_position(motor, -1.4); /* arm up */
    else if (i == 100) {
      /* turn */
      wb_motor_set_velocity(left_motor, -2);
      wb_motor_set_velocity(right_motor, 2);
    } else if (i == 140) {
      /* forward */
      wb_motor_set_velocity(left_motor, 10);
      wb_motor_set_velocity(right_motor, 10);
    } else if (i == 160) {
      /* turn */
      wb_motor_set_velocity(left_motor, -2);
      wb_motor_set_velocity(right_motor, 2);
    } else if (i == 200) {
      /* stops   */
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      wb_motor_set_position(motor, 0.0); /* arm down */
    } else if (i == 240)
      set_grip_position(OPEN_GRIP); /* open the gripper */
    else if (i == 260)
      wb_motor_set_position(motor, -1.4); /* arm up */

    if (i++ == 280) {
      i = 0;
    }
  }

  wb_robot_cleanup();

  return 0;
}
