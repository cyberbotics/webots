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
 * Description:  The controller of the robot from the world texture_change.
 */

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SPEED 10
#define TIME_STEP 64

int main() {
  WbDeviceTag camera, left_ds, right_ds, left_motor, right_motor;
  int backward_counter = 0; /* used to make the robot move backward for a while */
  int left_speed, right_speed;

  wb_robot_init();

  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  left_ds = wb_robot_get_device("ds0");
  right_ds = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(left_ds, TIME_STEP);
  wb_distance_sensor_enable(right_ds, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    wb_camera_get_image(camera); /* to refresh the camera image (maybe useless) */
    if (backward_counter == 0) {
      left_speed = SPEED;
      right_speed = SPEED;
      if (wb_distance_sensor_get_value(left_ds) + wb_distance_sensor_get_value(right_ds) > 300) {
        backward_counter = 300;
      }
    } else {
      left_speed = -SPEED;
      right_speed = -SPEED;
      backward_counter--;
    }
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }
  wb_robot_cleanup();
  return 0;
}
