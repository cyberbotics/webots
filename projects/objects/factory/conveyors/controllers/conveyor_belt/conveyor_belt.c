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

#include <stdlib.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <assert.h>
#include <stdio.h>

int main(int argc, char *argv[]) {
  wb_robot_init();
  assert(argc == 3);  // speed and timer excepted as argument.

  const int timestep = (int)wb_robot_get_basic_time_step();

  double speed;
  sscanf(argv[1], "%lf", &speed);

  double timer;
  sscanf(argv[2], "%lf", &timer);

  WbDeviceTag belt_motor = wb_robot_get_device("belt_motor");
  wb_motor_set_position(belt_motor, INFINITY);
  wb_motor_set_velocity(belt_motor, speed);

  while (wb_robot_step(timestep) != -1) {
    if (timer > 0 && wb_robot_get_time() >= timer) {
      wb_motor_set_velocity(belt_motor, 0.0);
      wb_robot_step(timestep);
      break;
    }
  }

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
