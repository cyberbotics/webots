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
#include <webots/motor.h>
#include <webots/robot.h>

// time in [ms] of a simulation step
#define TIME_STEP 64

#define MAX_SPEED 6.28

// entree point of the controller
int main(int argc, char **argv) {
  // initialise the Webots API
  wb_robot_init();

  // internal variables
  int i;
  WbDeviceTag ps[8];
  char ps_names[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  // get a handler to the motors and set target position to infinity (speed control)
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // initialise devices
  for (i = 0; i < 8; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  // feedback loop: step simulation until an exit event is received
  while (wb_robot_step(TIME_STEP) != -1) {
    // read sensors outputs
    double ps_values[8];
    for (i = 0; i < 8; i++)
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);

    // detect obsctacles
    bool right_obstacle = ps_values[0] > 80.0 || ps_values[1] > 80.0 || ps_values[2] > 80.0;
    bool left_obstacle = ps_values[5] > 80.0 || ps_values[6] > 80.0 || ps_values[7] > 80.0;

    // initialize motor speeds at 50% of MAX_SPEED.
    double left_speed = 0.5 * MAX_SPEED;
    double right_speed = 0.5 * MAX_SPEED;

    // modify speeds according to obstacles
    if (left_obstacle) {
      // turn right
      left_speed += 0.5 * MAX_SPEED;
      right_speed -= 0.5 * MAX_SPEED;
    } else if (right_obstacle) {
      // turn left
      left_speed -= 0.5 * MAX_SPEED;
      right_speed += 0.5 * MAX_SPEED;
    }

    // write actuators inputs
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  // cleanup the Webots API
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}
