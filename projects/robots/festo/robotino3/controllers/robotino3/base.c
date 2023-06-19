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
 * Description:   Implement the functions defined in base.h
 */

#include "base.h"

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define WHEEL_RADIUS 0.063                     // [m]
#define DISTANCE_WHEEL_TO_ROBOT_CENTRE 0.1826  // [m]
#define MAX_SPEED 8                            // [m/s]
#define DEMO_SPEED 2                           // [m/s]
#define OBSTACLE_THRESHOLD 0.20                // [m]

WbDeviceTag wheels[3];

double actualSpeed[3] = {0.0, 0.0, 0.0};
double targetSpeed[3] = {0.0, 0.0, 0.0};
double maxAcceleration[3] = {10.0, 6.0, 20.0};

void base_reset() {
  targetSpeed[0] = 0.0;
  targetSpeed[1] = 0.0;
  targetSpeed[2] = 0.0;
}

void base_forwards() {
  base_set_speeds(DEMO_SPEED / 2.0, 0, 0);
}

void base_backwards() {
  base_set_speeds(-DEMO_SPEED / 2.0, 0, 0);
}

void base_turn_left() {
  base_set_speeds(0, 0, 3.0 * DEMO_SPEED / 2.0);
}

void base_turn_right() {
  base_set_speeds(0, 0, -3.0 * DEMO_SPEED / 2.0);
}

void base_strafe_left() {
  base_set_speeds(0, DEMO_SPEED / 2.0, 0);
}

void base_strafe_right() {
  base_set_speeds(0, -DEMO_SPEED / 2.0, 0);
}

void base_apply_speeds(double vx, double vy, double omega) {
  vx /= WHEEL_RADIUS;
  vy /= WHEEL_RADIUS;
  omega *= DISTANCE_WHEEL_TO_ROBOT_CENTRE / WHEEL_RADIUS;
  wb_motor_set_velocity(wheels[0], vy - omega);
  // cos(angle between wheel and movement axis)
  wb_motor_set_velocity(wheels[1], -sqrt(0.75) * vx - 0.5 * vy - omega);
  wb_motor_set_velocity(wheels[2], sqrt(0.75) * vx - 0.5 * vy - omega);
}

void base_accelerate() {
  const double time_step = wb_robot_get_basic_time_step();
  double maxSteps = 0;
  for (int i = 3; i--;) {
    double stepsNeeded = fabs(targetSpeed[i] - actualSpeed[i]);
    stepsNeeded /= maxAcceleration[i] * (time_step / 1000.0);
    if (stepsNeeded > maxSteps)
      maxSteps = stepsNeeded;
  }
  if (maxSteps < 1)
    maxSteps = 1;
  for (int i = 3; i--;) {
    actualSpeed[i] += (targetSpeed[i] - actualSpeed[i]) / maxSteps;
  }
  base_apply_speeds(actualSpeed[0], actualSpeed[1], actualSpeed[2]);
}

void base_set_speeds(double vx, double vy, double omega) {
  targetSpeed[0] = vx;
  targetSpeed[1] = vy;
  targetSpeed[2] = omega;
}

void base_braitenberg_avoidance(const double *sensors_values) {
  // Simple obstacle avoidance algorithm
  // - obstacle in front
  if (sensors_values[0] < OBSTACLE_THRESHOLD)
    base_backwards();
  // - obstacle on left side
  else if (sensors_values[2] < OBSTACLE_THRESHOLD)
    base_strafe_right();
  // - obstacle on right side
  else if (sensors_values[7] < OBSTACLE_THRESHOLD)
    base_strafe_left();
  // - obstacle behind
  if (sensors_values[4] < OBSTACLE_THRESHOLD || sensors_values[5] < OBSTACLE_THRESHOLD)
    base_forwards();
  // - obstacle in front left
  else if (sensors_values[1] < OBSTACLE_THRESHOLD)
    base_turn_right();
  // - obstacle in front right
  else if (sensors_values[8] < OBSTACLE_THRESHOLD)
    base_turn_left();
  // - obstacle in rear left
  else if (sensors_values[3] < OBSTACLE_THRESHOLD)
    base_turn_right();
  // - obstacle in rear right_wheel
  else if (sensors_values[6] < OBSTACLE_THRESHOLD)
    base_turn_left();
  // - no obstacle
  else
    base_forwards();
}
