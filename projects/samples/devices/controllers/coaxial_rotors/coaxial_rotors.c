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
 * Description:  This controller shows how to use the Propeller node
 *               The torque effects of the two coaxial rotors cancell out
 *               and hence allows yaw stabilization
 *               The green helicopter in propeller.wbt also manages to stabilize
 *               its altitude by means of a simple velocity control based on GPS measurement
 */

#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define HELIX_VELOCITY 100.0
#define WEIGHT_FORCE 16.3465011
#define TARGET_ALTITUDE 2.0
#define LABEL_X 0.05
#define LABEL_Y 0.95
#define GREEN 0x008800

int main() {
  wb_robot_init();
  const int time_step = wb_robot_get_basic_time_step();

  // GPS and inertial unit
  const WbDeviceTag gps = wb_robot_get_device("gps");
  const WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  wb_gps_enable(gps, time_step);
  wb_inertial_unit_enable(inertial_unit, time_step);

  // helixes motion
  const WbDeviceTag upper_motor = wb_robot_get_device("upper_motor");
  const WbDeviceTag lower_motor = wb_robot_get_device("lower_motor");
  const WbDeviceTag tail_motor = wb_robot_get_device("tail_motor");
  wb_motor_set_position(upper_motor, INFINITY);
  wb_motor_set_position(lower_motor, INFINITY);
  wb_motor_set_position(tail_motor, INFINITY);
  double velocity = HELIX_VELOCITY + 1.0;
  wb_motor_set_velocity(upper_motor, -velocity);
  wb_motor_set_velocity(lower_motor, velocity);
  wb_motor_set_velocity(tail_motor, 15.0);

  char buffer[50];

  while (wb_robot_step(time_step) != -1) {
    const double altitude = wb_gps_get_values(gps)[2];
    const double yaw = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit)[2];
    sprintf(buffer, "Altitude: %1.1f m", altitude);
    wb_supervisor_set_label(5, buffer, LABEL_X, LABEL_Y, 0.07, GREEN, 0, "Arial");
    sprintf(buffer, "Yaw: %1.1f rad", yaw);
    wb_supervisor_set_label(6, buffer, LABEL_X, LABEL_Y - 0.03, 0.07, GREEN, 0, "Arial");
    const double ratio = 1.0 - altitude / TARGET_ALTITUDE;
    velocity = HELIX_VELOCITY + ratio;
    wb_motor_set_velocity(upper_motor, -velocity);
    wb_motor_set_velocity(lower_motor, velocity);
  }

  return 0;
}
