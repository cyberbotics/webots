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
 * Description:   Simple hello controller
 */

#include <webots/device.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16

void my_step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

int main(int argc, char **argv) {
  wb_robot_init();

  // list devices
  int n_devices = wb_robot_get_number_of_devices();
  int i;
  printf("Available devices:\n");
  for (i = 0; i < n_devices; i++) {
    WbDeviceTag tag = wb_robot_get_device_by_index(i);
    const char *name = wb_device_get_name(tag);
    printf(" Device #%d name = %s\n", i, name);
  }

  WbDeviceTag l_arm_shx = wb_robot_get_device("LArmShx");
  WbDeviceTag r_arm_shx = wb_robot_get_device("RArmShx");
  WbDeviceTag r_arm_elx = wb_robot_get_device("RArmElx");

  double l_arm_shx_target = -1.396;
  double r_arm_shx_target = -0.77;

  int n_steps_to_achieve__target = 1000 / TIME_STEP;  // 1 second
  for (i = 0; i < n_steps_to_achieve__target; i++) {
    double ratio = (double)i / n_steps_to_achieve__target;
    wb_motor_set_position(l_arm_shx, l_arm_shx_target * ratio);
    wb_motor_set_position(r_arm_shx, r_arm_shx_target * ratio);
    my_step();
  }

  double initTime = wb_robot_get_time();
  while (true) {
    double time = wb_robot_get_time() - initTime;
    wb_motor_set_position(r_arm_elx, 0.3 * sin(5 * time) - 0.3);
    my_step();
  };

  return EXIT_FAILURE;
}
