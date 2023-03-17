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
 * Description:  This robot controller generates a rotational motion whose
 *               sense periodically changes
 */

#include <webots/motor.h>
#include <webots/robot.h>

int main() {
  wb_robot_init();
  WbDeviceTag motor = wb_robot_get_device("rotational motor");
  wb_motor_set_position(motor, INFINITY);
  int time_step = wb_robot_get_basic_time_step();
  int time = 0;
  while (wb_robot_step(time_step) != -1) {
    wb_motor_set_velocity(motor, 2.0 * sin(0.004 * time));
    time++;
  }
  return 0;
}
