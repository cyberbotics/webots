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
 * Description:  Gears example
 */

#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 8

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag motor = wb_robot_get_device("rotational motor");
  wb_motor_set_position(motor, INFINITY);
  wb_motor_set_velocity(motor, 0.2);

  WbDeviceTag sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(sensor, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    double pos = wb_position_sensor_get_value(sensor);

    if (pos >= M_PI / 4)
      wb_motor_set_velocity(motor, -0.2);
    if (pos <= -M_PI / 4)
      wb_motor_set_velocity(motor, 0.2);
  }

  wb_robot_cleanup();

  return 0;
}
