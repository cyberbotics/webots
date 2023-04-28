#pragma once

/*
 * Copyright 2022 Bitcraze AB
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
 *  ...........       ____  _ __
 *  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 *  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 *  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 *
 * @file pid_controller.h
 * Description:  PID controller header file
 * Author:       Kimberly McGuire (Bitcraze AB)
 */

typedef struct motor_power_s {
  double m1;
  double m2;
  double m3;
  double m4;
} motor_power_t;

typedef struct control_commands_s {
  double roll;
  double pitch;
  double yaw;
  double altitude;
} control_commands_t;

typedef struct desired_state_s {
  double roll;
  double pitch;
  double yaw_rate;
  double altitude;
  double vx;
  double vy;
} desired_state_t;

typedef struct actual_state_s {
  double roll;
  double pitch;
  double yaw_rate;
  double altitude;
  double vx;
  double vy;
} actual_state_t;

typedef struct gains_pid_s {
  double kp_att_rp;
  double kd_att_rp;
  double kp_att_y;
  double kd_att_y;
  double kp_vel_xy;
  double kd_vel_xy;
  double kp_z;
  double kd_z;
  double ki_z;
} gains_pid_t;

float constrain(float value, const float minVal, const float maxVal);
void init_pid_attitude_fixed_height_controller();

void pid_attitude_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                          double dt, motor_power_t *motorCommands);

void pid_velocity_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                          double dt, motor_power_t *motorCommands);

void pid_fixed_height_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid, double dt,
                                 control_commands_t *control_commands);

void motor_mixing(control_commands_t control_commands, motor_power_t *motorCommands);

void pid_attitude_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid, double dt,
                             control_commands_t *control_commands);

void pid_horizontal_velocity_controller(actual_state_t actual_state, desired_state_t *desired_state, gains_pid_t gains_pid,
                                        double dt);