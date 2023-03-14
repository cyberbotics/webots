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
 * Description:  Default controller of the iRobot's wall accessory
 */

#include <stdio.h>
#include <stdlib.h>

#include <webots/emitter.h>
#include <webots/robot.h>

static void step() {
  int time_step = wb_robot_get_basic_time_step();
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

int main(int argc, char **argv) {
  wb_robot_init();

  printf("Default controller of the iRobot Create Wall started...\n");

  WbDeviceTag emitter = wb_robot_get_device("emitter");

  char dummy_data = 0xFF;

  // send data every step
  while (true) {
    wb_emitter_send(emitter, &dummy_data, 1);
    step();
  }

  return EXIT_SUCCESS;
}
