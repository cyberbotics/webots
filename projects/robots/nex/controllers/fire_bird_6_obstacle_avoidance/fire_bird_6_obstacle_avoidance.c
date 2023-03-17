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
 * Description:   Demo code for obstacle avoidance on Firebird 6 robot
 * Author:        Anant Malewar; Nex Robotics
 */

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <stdio.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();

  printf("Fire Bird 6 controller for obstacle avoidance\n");

  int i;
  WbDeviceTag ps[8], left_motor, right_motor;

  char ps_names[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

  for (i = 0; i < 8; ++i) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    double ps_values[8];
    for (i = 0; i < 8; ++i) {
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
      printf("%f ", ps_values[i]);
    }
    printf("\n");

    double threshold = 0.25;

    bool left_obstacle = ps_values[1] < threshold || ps_values[0] < threshold;
    bool right_obstacle = ps_values[3] < threshold || ps_values[4] < threshold;
    bool front_obstacle = ps_values[2] < threshold;

    // init speeds
    double left_speed = 1;
    double right_speed = 1;

    // modify speeds according to obstacles
    if (front_obstacle) {
      // turn back, but slightly right to not block the robot
      left_speed -= 1.0;
      right_speed -= 2.0;
    } else if (left_obstacle) {
      // turn right
      left_speed += 2.0;
      right_speed -= 2.0;
    } else if (right_obstacle) {
      // turn left
      left_speed -= 2.0;
      right_speed += 2.0;
    }

    // write actuators inputs
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  };

  wb_robot_cleanup();

  return 0;
}
