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
 * Description:  The controller for the Lego Mindstorm robot which follows a
 *               line on the ground and avoids obstacles.
 */

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#define TIME_STEP 64

int main() {
  double leftSpeed, rightSpeed;

  wb_robot_init();

  WbDeviceTag leftBumper = wb_robot_get_device("S1");
  WbDeviceTag groundSensor = wb_robot_get_device("S2");
  WbDeviceTag rightBumper = wb_robot_get_device("S3");

  /* We use this counter to synchronise the different parts of the avoidance movement.*/
  int avoidance_counter = 0;

  wb_touch_sensor_enable(leftBumper, TIME_STEP);
  wb_touch_sensor_enable(rightBumper, TIME_STEP);
  wb_distance_sensor_enable(groundSensor, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * If we are not avoiding an obstacle we check first the bumpers
     * to know if we have bumped into one and then the groundSensor to
     * know in which direction we should move.
     */
    if (avoidance_counter == 0) {
      if (wb_touch_sensor_get_value(leftBumper) > 0 || wb_touch_sensor_get_value(rightBumper) > 0) {
        leftSpeed = -0.6;
        rightSpeed = -1;
        avoidance_counter = 1000;
      } else if (wb_distance_sensor_get_value(groundSensor) > 43) {
        leftSpeed = 1;
        rightSpeed = 0.2;
      } else {
        leftSpeed = 0.2;
        rightSpeed = 1;
      }
    } else {
      /*
       * If we are avoiding, the movement is seperated into three
       * different parts. First we move back from the obstacle,
       * slightly turning, then we move straight forward in order to
       * avoid the obstacle. Finall we keep moving forward wut
       * slightly turning in order to find the line.
       */
      if (avoidance_counter > 800) {
        leftSpeed = -0.6;
        rightSpeed = -1;
      } else if (avoidance_counter > 600) {
        leftSpeed = 1;
        rightSpeed = 1;
      } else if (avoidance_counter > 70) {
        leftSpeed = 0.7;
        rightSpeed = 1;
        if (wb_distance_sensor_get_value(groundSensor) > 43)
          avoidance_counter = 1;
      } else {
        leftSpeed = 1;
        rightSpeed = 1;
        if (wb_distance_sensor_get_value(groundSensor) > 43)
          avoidance_counter = 1;
      }
      avoidance_counter--;
    }
    wb_motor_set_velocity(left_motor, leftSpeed);
    wb_motor_set_velocity(right_motor, rightSpeed);
  }

  wb_robot_cleanup();

  return 0;
}
