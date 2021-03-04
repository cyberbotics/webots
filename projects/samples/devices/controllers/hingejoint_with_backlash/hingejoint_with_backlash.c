/*
 * Copyright 1996-2021 Cyberbotics Ltd.
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
 * Description: The axis of a gear train lags behind the rotor that drives it.
 */

#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 16
#define SPEED 0.2f

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag rotor = wb_robot_get_device("rotor motor");

  WbDeviceTag rotorSensor = wb_robot_get_device("rotor sensor");
  wb_position_sensor_enable(rotorSensor, TIME_STEP);

  wb_motor_set_position(rotor, INFINITY);
  wb_motor_set_velocity(rotor, SPEED);

  while (wb_robot_step(TIME_STEP) != -1) {
    double pos = wb_position_sensor_get_value(rotorSensor);
    if (pos > 1.0471)
      wb_motor_set_velocity(rotor, -SPEED);
    if (pos < -1.0471)
      wb_motor_set_velocity(rotor, SPEED);
  }

  wb_robot_cleanup();

  return 0;
}
