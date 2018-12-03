/*
 * Copyright 1996-2018 Cyberbotics Ltd.
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

/*!
 * (c) 2006 - 2008 EPFL, Lausanne, Switzerland
 * Thomas Lochmatter
 */

#include "odometry_goto.h"

#include <math.h>
#include <stdlib.h>
#include <webots/robot.h>
#define PI 3.14159265358979

void odometry_goto_init() {
}

void odometry_goto_start(struct sOdometryGoto *og, struct sOdometryTrack *ot) {
  og->track = ot;
  og->configuration.speed_min = 1.;
  og->state.goal_x = 0.;
  og->state.goal_y = 0.;
  og->state.goal_theta = 0.;
  og->result.speed_left = 0;
  og->result.speed_right = 0;
  og->result.atgoal = 1;

  return;
}

void odometry_goto_set_goal(struct sOdometryGoto *og, float goal_x, float goal_y, float goal_theta) {
  og->state.goal_x = goal_x;
  og->state.goal_y = goal_y;
  og->state.goal_theta = goal_theta;
  og->result.atgoal = 0;

  return;
}

void odometry_goto_step(struct sOdometryGoto *og) {
  // get current position
  float curPos[3] = {og->track->result.x, og->track->result.y, og->track->result.theta};
  float goalPos[3] = {og->state.goal_x, og->state.goal_y, og->state.goal_theta};

  float dx = goalPos[0] - curPos[0];
  float dy = goalPos[1] - curPos[1];

  // controller parameters, an initial choice for the values is given but might be changed
  float k_rho = 1.2;
  float k_alpha = 1.5;
  float k_beta = -0.35;

  // "v_c" is the robot's velocity in its longitudinal direction
  // the values range from -1000 to +1000
  // which corresponds approx. to max. 130mm/s
  float v_adapt = 1000 / 0.13;  // conversion-factor for speed in [m/s] to e-Puck speed units

  // "omega_c" is the robot's rotational speed around the vertical axis
  // (positiv for turns in counter-clockwise direction)
  // the value is defined to range from -2000 to 2000
  // representing turn rates of max. 270°/s
  float omega_adapt = 2000 / (270 * PI / 180);  // conversion-factor for turn rate in [rad/s] to e-Puck speed units

  // calculate current distance and angles to goal position
  float rho_c = sqrt(dx * dx + dy * dy);

  float alpha_c = atan2(dy, dx) - curPos[2];
  while (alpha_c > PI)  // to prevent alpha from getting too big
    alpha_c = alpha_c - 2 * PI;
  while (alpha_c < -PI)
    alpha_c = alpha_c + 2 * PI;

  float beta_c = -curPos[2] - alpha_c;
  while (beta_c > PI)  // to prevent beta from getting too big
    beta_c = beta_c - 2 * PI;
  while (beta_c < -PI)
    beta_c = beta_c + 2 * PI;

  // control law
  float v_c = k_rho * rho_c;
  float omega_c = k_alpha * alpha_c + k_beta * beta_c;

  // adapt SI values to e-Puck units
  float v_e = v_c * v_adapt;
  float omega_e = omega_c * omega_adapt;

  // finally record motor speed
  og->result.speed_left = (int)(v_e - omega_e / 2);
  og->result.speed_right = (int)(v_e + omega_e / 2);

  // Don't set speeds < MIN_SPEED (for accuracy reasons)
  if (abs(og->result.speed_left) < og->configuration.speed_min)
    og->result.speed_left = 0;
  if (abs(og->result.speed_right) < og->configuration.speed_min)
    og->result.speed_right = 0;

  // Termination condition
  if (((og->result.speed_left == 0) && (og->result.speed_right == 0)) || rho_c < 0.002)
    og->result.atgoal = 1;

  return;
}
