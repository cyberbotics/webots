/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Example controller that demonstrates the usage of "laser" DistanceSensor.
 *                Two laser spots appear where the laser beam hits the obstacles.
 *                These dots are visible in the camera images.
 */

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 64

int main() {
  // initialize webots stuff
  wb_robot_init();

  // get camera tag and initialize
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  WbDeviceTag laser0 = wb_robot_get_device("laser0");
  WbDeviceTag laser1 = wb_robot_get_device("laser1");
  wb_distance_sensor_enable(laser0, TIME_STEP);
  wb_distance_sensor_enable(laser1, TIME_STEP);

  // get a handler to the motors and set target position to infinity (speed control).
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // control loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // update image
    (void)wb_camera_get_image(camera);

    // turn around
    wb_motor_set_velocity(left_motor, -1);
    wb_motor_set_velocity(right_motor, 1);
  }

  wb_robot_cleanup();
  return 0;
}
