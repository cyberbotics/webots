/*
 * Copyright 1996-2020 Cyberbotics Ltd.
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
 * Description:   An example of a controller using the Track node to model a conveyor belt.
 */

#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();

  int timeStep = wb_robot_get_basic_time_step();
  WbDeviceTag motor = wb_robot_get_device("linear motor");

  double p = 0.0;
  while (wb_robot_step(timeStep) != -1) {
    p += 0.0005;
    wb_motor_set_position(motor, p);
  };

  wb_robot_cleanup();

  return 0;
}
