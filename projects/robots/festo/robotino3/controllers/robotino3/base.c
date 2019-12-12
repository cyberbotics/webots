/*
 * Copyright 1996-2019 Cyberbotics Ltd.
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
 * Description:   Implement the functions defined in base.h
 */

#include "base.h"

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>

#define WHEEL_DIAMETER 0.125                      // [m]
#define WHEEL_RADIUS_GAP 0.184                    // [m]
#define MAX_LIN_SPEED_KM_H 10                     // [km/h]
#define MAX_LIN_SPEED (MAX_LIN_SPEED_KM_H / 3.6)  // [m/s]
#define LIN_SPEED 1.1                             // [m/s]
#define OBSTACLE_THRESHOLD 0.20                   // [m]

WbDeviceTag wheels[3];

void base_reset() {
  const double speeds[3] = {0.0, 0.0, 0.0};
  base_set_wheel_speeds_helper(speeds);
}

void base_forwards() {
  double speeds[3] = {MAX_LIN_SPEED / 2.0, 0, 0};
  base_set_wheel_speeds_helper(speeds);
}

void base_backwards() {
  double speeds[3] = {-MAX_LIN_SPEED / 2.0, 0, 0};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_left() {
  double speeds[3] = {0, 0, -3.0 * MAX_LIN_SPEED / 2.0};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_right() {
  double speeds[3] = {0, 0, 3.0 * MAX_LIN_SPEED / 2.0};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_left() {
  double speeds[3] = {0, -MAX_LIN_SPEED / 2.0, 0};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_right() {
  double speeds[3] = {0, MAX_LIN_SPEED / 2.0, 0};
  base_set_wheel_speeds_helper(speeds);
}

void base_set_wheel_velocity(WbDeviceTag t, double velocity) {
  wb_motor_set_position(t, INFINITY);
  if (velocity > wb_motor_get_max_velocity(t))
    velocity = wb_motor_get_max_velocity(t);
  if (velocity < -wb_motor_get_max_velocity(t))
    velocity = -wb_motor_get_max_velocity(t);
  wb_motor_set_velocity(t, velocity);
}

void base_set_wheel_speeds_helper(double *speeds) {
  int i;
  double w_motor[3] = {0.0, 0.0, 0.0};
  double v_x = speeds[1];
  double v_y = speeds[0];
  double w = speeds[2];

  // Conversion matrix from paper, section 4:
  // http://ftp.itam.mx/pub/alfredo/ROBOCUP/SSLDocs/PapersTDPs/omnidrive.pdf
  w_motor[0] = -0.5 * v_x + 0.866 * v_y + WHEEL_RADIUS_GAP * w;
  w_motor[1] = -0.5 * v_x - 0.866 * v_y + WHEEL_RADIUS_GAP * w;
  w_motor[2] = v_x + WHEEL_RADIUS_GAP * w;

  for (i = 0; i < 3; i++)
    base_set_wheel_velocity(wheels[i], w_motor[i]);
}

void base_braitenberg_avoidance(double *sensors_values) {
  // Simple obstacle avoidance algorithm
  // - obstacle in front
  if (sensors_values[0] < OBSTACLE_THRESHOLD) {
    base_backwards();
  }
  // - obstacle on left side
  else if (sensors_values[2] < OBSTACLE_THRESHOLD) {
    base_strafe_right();
  }
  // - obstacle on right side
  else if (sensors_values[7] < OBSTACLE_THRESHOLD) {
    base_strafe_left();
  }
  // - obstacle behind
  if (sensors_values[4] < OBSTACLE_THRESHOLD || sensors_values[5] < OBSTACLE_THRESHOLD) {
    base_forwards();
  }
  // - obstacle in front left
  else if (sensors_values[1] < OBSTACLE_THRESHOLD) {
    base_turn_right();
  }
  // - obstacle in front right
  else if (sensors_values[8] < OBSTACLE_THRESHOLD) {
    base_turn_left();
  }
  // - obstacle in rear left
  else if (sensors_values[3] < OBSTACLE_THRESHOLD) {
    base_turn_right();
  }
  // - obstacle in rear right_wheel
  else if (sensors_values[6] < OBSTACLE_THRESHOLD) {
    base_turn_left();
  }
  // - no obstacle
  else {
    base_forwards();
  }
}
