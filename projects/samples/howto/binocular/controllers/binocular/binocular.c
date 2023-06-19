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
 * Description:  An example of use of two cameras to produce stereoscopy.
 */

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SPEED 6
#define TIME_STEP 64

int main() {
  WbDeviceTag ds0, ds1, camera0, camera1, left_motor, right_motor;

  wb_robot_init();

  // initialize distance sensors
  ds0 = wb_robot_get_device("ds0");
  ds1 = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

  // initialize cameras
  camera0 = wb_robot_get_device("camera0");
  camera1 = wb_robot_get_device("camera1");
  wb_camera_enable(camera0, 2 * TIME_STEP);
  wb_camera_enable(camera1, 2 * TIME_STEP);

  // initialize the motors and set target position to infinity (speed control).
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    // This is used to refresh the cameras
    wb_camera_get_image(camera0);
    wb_camera_get_image(camera1);

    double ds0_value = wb_distance_sensor_get_value(ds0);
    double ds1_value = wb_distance_sensor_get_value(ds1);

    double left_speed, right_speed;
    if (ds1_value > 500) {
      /*
       * If both distance sensors are detecting something, this means that
       * we are facing a wall. In this case we need to move backwards.
       */
      if (ds0_value > 200) {
        left_speed = -SPEED;
        right_speed = -SPEED / 2;
      } else {
        /*
         * We turn proportionnaly to the sensors value because the
         * closer we are from the wall, the more we need to turn.
         */
        left_speed = -ds1_value / 100;
        right_speed = (ds0_value / 100) + 0.5;
      }
    } else if (ds0_value > 500) {
      left_speed = (ds1_value / 100) + 0.5;
      right_speed = -ds0_value / 100;
    } else {
      /*
       * If nothing was detected we can move forward at maximal speed.
       */
      left_speed = SPEED;
      right_speed = SPEED;
    }
    // Set the motor speeds.
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }
  return 0;
}
