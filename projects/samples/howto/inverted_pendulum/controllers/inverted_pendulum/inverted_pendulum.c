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
 * Description:  A controller using the PID technic to control an inverted
 *               pendulum.
 */

#include <math.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define MAX_FORCE 40
#define CENTERING_ANGLE 0.7
#define CENTERING_FORCE 4
#define TIME_STEP 32

static WbDeviceTag horizontal_motor, horizontal_position_sensor, hip;
static double previous_position, differential, integral_sum;

static void initialize() {
  wb_robot_init();
  horizontal_motor = wb_robot_get_device("horizontal_motor");
  horizontal_position_sensor = wb_robot_get_device("horizontal position sensor");
  hip = wb_robot_get_device("hip");
  wb_position_sensor_enable(horizontal_position_sensor, TIME_STEP);
  wb_position_sensor_enable(hip, TIME_STEP);

  previous_position = wb_position_sensor_get_value(hip);
  differential = 0;
  integral_sum = 0;
}

static void run() {
  double position = wb_position_sensor_get_value(hip);
  differential = position - previous_position;
  integral_sum += position;

  /*
   * This is the PID formula : position, integral, differential. The
   * coefficients for each variable were heuristically found. They are still
   * not the best ones.
   */
  double power = (50 * position / 1) + (4 * differential / 1) + (2 * integral_sum / 1);

  /*
   * We use this part in order to try to recenter the robot.
   */
  if (fabs(position) < CENTERING_ANGLE) {
    if (wb_position_sensor_get_value(horizontal_position_sensor) > 0)
      power -= CENTERING_FORCE;
    else
      power += CENTERING_FORCE;
  }

  power = power < MAX_FORCE ? power : MAX_FORCE;
  power = power > -MAX_FORCE ? power : -MAX_FORCE;

  wb_motor_set_force(horizontal_motor, power);

  previous_position = position;
}

int main() {
  initialize();
  while (1) {
    wb_robot_step(TIME_STEP);
    run();
  }
  return 0;
}
