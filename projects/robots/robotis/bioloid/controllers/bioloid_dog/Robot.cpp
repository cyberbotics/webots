// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Robot.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

/*
 * List of motors
 */
const char *Robot::MOTOR_NAMES[MAX_MOTORS + 1] = {"pelvis",
                                                  "front_left_1",
                                                  "front_right_1",
                                                  "front_left_2",
                                                  "front_right_2",
                                                  "front_left_3",
                                                  "front_right_3",
                                                  "back_left_1",
                                                  "back_right_1",
                                                  "back_left_2",
                                                  "back_right_2",
                                                  "back_left_3",
                                                  "back_right_3",
                                                  "neck_1",
                                                  "neck_2",
                                                  "head",
                                                  NULL};

/*
 * List of possible walking gait
 */
const char *Robot::gait_name[7] = {"trot", "walk", "gallop(transverse)", "canter", "pace", "bound", "pronk"};

const double Robot::gait_phase_shift[7][4] = {
  {0, 0.5, 0, 0.5},      // trot
  {0, 0.5, 0.25, 0.75},  // walk
  {0, 0.1, 0.6, 0.5},    // gallop  (transverse)
  {0, 0.3, 0, 0.7},      // canter
  {0, 0.5, 0.5, 0},      // pace
  {0, 0, 0.5, 0.5},      // bound
  {0, 0, 0, 0}           // pronk
};

const int Robot::gait_setup[4][2] = {{FRONT_LEFT_1, FRONT_LEFT_3},
                                     {FRONT_RIGHT_1, FRONT_RIGHT_3},
                                     {BACK_RIGHT_1, BACK_RIGHT_3},
                                     {BACK_LEFT_1, BACK_LEFT_3}};

Robot::Robot(const char *name) : _name(name), _controlStep(SIMULATION_STEP_DURATION), _stepCount(0), _camera(0) {
  int i = 0;
  for (i = 0; Robot::MOTOR_NAMES[i]; i++) {
    motors[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    assert(motors[i]);
    position_sensor[i] = wb_motor_get_position_sensor(motors[i]);
    assert(position_sensor[i]);
    wb_position_sensor_enable(position_sensor[i], (int)_controlStep);
  }

  // set up gps
  _gps = wb_robot_get_device("gps");
  wb_gps_enable(_gps, (unsigned int)_controlStep);
  printf("gps device tag is %d\n", (int)_gps);
}

/*
 * Set motor position
 */
void Robot::setMotorPosition(int motorId, double value) {
  wb_motor_set_position(motors[motorId], value);
}

/*
 * Get motor position
 */
double Robot::getMotorPosition(int motorId) {
  return wb_position_sensor_get_value(position_sensor[motorId]);
}

/*
 * Enable the 'motor_get_position' function for the provided motor
 */
void Robot::enableMotorPosition(int motorId) {
  wb_position_sensor_enable(position_sensor[motorId], (int)_controlStep);
}

/*
 * Init Camera
 */
void Robot::initCamera() {
  _camera = wb_robot_get_device("camera");
  wb_camera_enable(_camera, (int)_controlStep);
}

/*
 * Run simulation for X seconds
 */
void Robot::wait(double x) {
  // number of iteration
  double num = x / (_controlStep / 1000);
  for (int i = 0; i < num; i++)
    wb_robot_step((int)_controlStep);
}

/*
 * Set the robot into walking position
 */
void Robot::standing() {
  setMotorPosition(NECK_1, -M_PI / 2);
  setMotorPosition(FRONT_LEFT_2, M_PI / 2);
  setMotorPosition(FRONT_RIGHT_2, -M_PI / 2);
  setMotorPosition(BACK_LEFT_2, M_PI / 2);
  setMotorPosition(BACK_RIGHT_2, -M_PI / 2);

  wait(1);
}

/*
 * Compute the position of the three motors according to the time
 */
void Robot::computeWalkingPosition(double *motorsPosition, double t, double gait_freq, int gait_type, int legId,
                                   double stride_length_factor, bool backward) {
  // proceed to reverse kinematic to compute leg position
  double freq = gait_freq;

  // compute modulo
  int n = (int)(t / (1 / freq));
  t = t - n * (1 / freq);

  // reverse time sequence for backward walk
  if (backward)
    t = (1 / freq) - t;

  // ellipsoid parameters
  double a = 0.95 * L1 * stride_length_factor;
  double h = 0;
  double k = -(L1 + L2 / 2);
  double b = -k - sqrt(L1 * L1 + L2 * L2);

  // compute ellipsoid points
  double x = h + a * cos(2 * M_PI * freq * t + gait_phase_shift[gait_type][legId] * 2 * M_PI);
  double y = k + b * sin(2 * M_PI * freq * t + gait_phase_shift[gait_type][legId] * 2 * M_PI);

  // compute angle A2
  double A2 = acos((x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2));

  // compute angle A1
  double A1 = acos(((L1 + L2 * cos(A2)) * x - (-L2 * sin(A2)) * y) / (pow(L1 + L2 * cos(A2), 2) + pow(-L2 * sin(A2), 2)));

  // subtract 2PI
  A1 = M_PI / 2 - A1;

  motorsPosition[0] = A1;
  motorsPosition[1] = A2;
}

/*
 *
 */
void Robot::interactive_walk() {
  int gait_type = 0, i = 0;
  int key = -1;

  bool backward = false;  // is robot walking backward

  // stride length parameters
  double slf = 1, slf_min = 0, slf_max = 1;
  double stride_length_factor[4] = {slf, slf, slf, slf};

  // frequency parameters
  double freq_min = 0.4, freq_max = 2;
  double freq = 1.5, freq_offset = 0.2;

  // turn amount parameters
  double ta_factor[4] = {0, 0, 0, 0};
  double ta_min = -0.6, ta_max = 0.6;
  double ta_offset = 0.6;
  // double ta_min = -1, ta_max = 1;
  // double ta_offset = 0.1;
  // Nota: It reveals the a turn offset of 0.6 gives good result

  bool display_info = false;

  while (true) {
    // get keyboard
    const int prev_key = key;
    key = wb_keyboard_get_key();

    if (key != prev_key) {
      // update var according to 'key' value
      switch (key) {
        case WB_KEYBOARD_RIGHT:
          for (i = 0; i < 4; i++) {
            if (i == 0 || i == 3)
              ta_factor[i] += ta_offset;
            else
              ta_factor[i] -= ta_offset;
            ta_factor[i] = ta_factor[i] > ta_max ? ta_max : (ta_factor[i] < ta_min ? ta_min : ta_factor[i]);
          }
          display_info = true;
          break;

        case WB_KEYBOARD_LEFT:
          for (i = 0; i < 4; i++) {
            if (i == 0 || i == 3)
              ta_factor[i] -= ta_offset;
            else
              ta_factor[i] += ta_offset;
            ta_factor[i] = ta_factor[i] > ta_max ? ta_max : (ta_factor[i] < ta_min ? ta_min : ta_factor[i]);
          }
          display_info = true;
          break;

        case 'F':
          backward = false;
          freq = 1.5;
          display_info = true;
          break;
        case 'B':
          backward = true;
          freq = 0.9;
          display_info = true;
          break;

        // case WB_KEYBOARD_UP:
        case 'S':
          slf += 0.1;
          display_info = true;
          break;

        // case WB_KEYBOARD_DOWN:
        case 'A':
          slf -= 0.1;
          display_info = true;
          break;

        case 'Q':
          freq += freq_offset;
          display_info = true;
          break;

        case 'W':
          freq -= freq_offset;
          display_info = true;
          break;
        default:
          break;
      }

      // bound value
      freq = freq > freq_max ? freq_max : (freq < freq_min ? freq_min : freq);
      slf = slf > slf_max ? slf_max : (slf < slf_min ? slf_min : slf);

      // update stride_length_factor for each leg
      for (i = 0; i < 4; i++) {
        stride_length_factor[i] = ta_factor[i] + slf;
        // bound stride length
        stride_length_factor[i] =
          stride_length_factor[i] > slf_max ? slf_max : (stride_length_factor[i] < slf_min ? slf_min : stride_length_factor[i]);
      }
    }

    // display walking gait informations
    if (display_info) {
      // display freq + turn amount
      printf("freq:%f, slf:%f, backward:%d\n", freq, slf, backward);
      for (i = 0; i < 4; i++)
        printf("leg[%d] ta:%f, slf:%f \n", i, ta_factor[i], stride_length_factor[i]);
      display_info = false;
    }

    double motorPositions[2] = {0, 0};
    // compute motors position for each legs
    for (int legId = 0; legId < 4; legId++) {
      computeWalkingPosition(motorPositions, _stepCount * (_controlStep / 1000), freq, gait_type, legId,
                             stride_length_factor[legId], backward);
      setMotorPosition(gait_setup[legId][0], motorPositions[0]);
      setMotorPosition(gait_setup[legId][1], motorPositions[1]);
    }

    // simulator step
    wb_robot_step((unsigned int)_controlStep);
    _stepCount++;
  }
}
