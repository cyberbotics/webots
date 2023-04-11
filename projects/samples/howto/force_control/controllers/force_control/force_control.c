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
 * Description:  Example of spring and dampers simulation implemented with force control
 */

#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <stdio.h>

#define CONTROL_STEP 4
#define SPRING_CONSTANT 40.0
#define DAMPING_CONSTANT 0.2

int main() {
  /* initialize Webots */
  wb_robot_init();

  /* get motor and position sensor and enable position sensor */
  WbDeviceTag motor = wb_robot_get_device("slider");
  WbDeviceTag position_sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(position_sensor, CONTROL_STEP);

  /* forever... */
  while (1) {
    /* move red and blue blocks appart */
    printf("Reset\n");
    wb_motor_set_position(motor, 0.3);
    int i;
    for (i = 0; i < 500; i++) /* for 2 seconds */
      wb_robot_step(CONTROL_STEP);

    /* spring simulation */
    printf("Start spring simulation\n");
    double previous_pos = 0.0;
    for (i = 0; i < 2500; i++) { /* for 10 seconds */
      double pos = wb_position_sensor_get_value(position_sensor);

      /* compute velocity in [m/s] */
      double vel = (pos - previous_pos) / (CONTROL_STEP / 1000.0);
      previous_pos = pos;

      /* spring force */
      double sf = -SPRING_CONSTANT * pos;

      /* damping force */
      double df = -DAMPING_CONSTANT * vel;

      /* add spring and damping forces */
      wb_motor_set_force(motor, sf + df);

      /* run physics simulation and return */
      wb_robot_step(CONTROL_STEP);
    }
  }

  return 0;
}
