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
 * Description: simple example of linear motor position control
 */

#include <math.h>
#include <webots/motor.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();
  double time_step = wb_robot_get_basic_time_step();
  WbDeviceTag motor = wb_robot_get_device("linear motor");
  double target = 0;
  int counter = 0;
  /* This simple algorithm moves the linear motor to position 0   */
  /* and then progresses by steps of 2 cm until it reaches 20 cm. */
  /* Once done, it comes back to position 0 and restarts.         */
  /* The duration of each step is 100 * time_step = 1.6 second    */
  /* (assuming time_step = 16 milliseconds)                       */
  while (wb_robot_step(time_step) != -1) {
    wb_motor_set_position(motor, target);
    if (counter++ == 100) { /* 1.6 seconds elapsed */
      target += 0.02;       /* 2 cm */
      if (target > 0.2)     /* 20 cm */
        target = 0;
      counter = 0;
    }
  };
  wb_robot_cleanup();
  return 0;
}
