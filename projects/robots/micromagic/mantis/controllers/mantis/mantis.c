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

#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  wb_robot_init();

  int time_step = (int)wb_robot_get_basic_time_step();

  // Meaning of the motor charcters:
  // - 'R': Right / 'L': Left
  // - 'A': Front / 'M': Middle / 'P': Rear
  // - 'C': Base / 'F': Shoulder / 'T': Knee
  WbDeviceTag motors[18] = {wb_robot_get_device("RPC"), wb_robot_get_device("RPF"), wb_robot_get_device("RPT"),
                            wb_robot_get_device("RMC"), wb_robot_get_device("RMF"), wb_robot_get_device("RMT"),
                            wb_robot_get_device("RAC"), wb_robot_get_device("RAF"), wb_robot_get_device("RAT"),
                            wb_robot_get_device("LPC"), wb_robot_get_device("LPF"), wb_robot_get_device("LPT"),
                            wb_robot_get_device("LMC"), wb_robot_get_device("LMF"), wb_robot_get_device("LMT"),
                            wb_robot_get_device("LAC"), wb_robot_get_device("LAF"), wb_robot_get_device("LAT")};

  // The parameters for a simple walking gate have been found empirically.

  // frequency [Hz]
  const double f = 0.5;

  // amplitude [rad]
  const double aC = 0.25;  // Amplitude for the "base" motors.
  const double aF = 0.2;   // Amplitude for the "shoulder" motors.
  const double aT = 0.05;  // Amplitude for the "knee" motors.
  const double a[18] = {aC, aF, -aT, -aC, -aF, aT, aC, aF, -aT, aC, -aF, aT, -aC, aF, -aT, aC, -aF, aT};

  // phase [s]
  double pC = 0.0;  // Phase for the "base" motors.
  double pF = 2.0;  // Phase for the "shoulder" motors.
  double pT = 2.5;  // Phase for the "knee" motors.
  const double p[18] = {pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT, pC, pF, pT};

  // offset [rad]
  const double dC = 0.6;   // Offset for the "base" motors.
  const double dF = 0.8;   // Offset for the "shoulder" motors.
  const double dT = -2.4;  // Offset for the "knee" motors.
  const double d[18] = {-dC, dF, dT, 0.0, dF, dT, dC, dF, dT, dC, dF, dT, 0.0, dF, dT, -dC, dF, dT};

  while (wb_robot_step(time_step) != -1) {
    double time = wb_robot_get_time();
    int i;
    for (i = 0; i < 18; ++i)  // Apply a sinuosidal function for each motor.
      wb_motor_set_position(motors[i], a[i] * sin(2.0 * M_PI * f * time + p[i]) + d[i]);
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
