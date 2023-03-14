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
 * Description: simple example of motor position control for BallJoint
 */

#include <math.h>
#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();
  double time_step = wb_robot_get_basic_time_step();
  WbDeviceTag motor1 = wb_robot_get_device("motor 1");
  WbDeviceTag motor2 = wb_robot_get_device("motor 2");
  WbDeviceTag motor3 = wb_robot_get_device("motor 3");

  int counter = 0;
  while (wb_robot_step(time_step) != -1) {
    counter++;
    if (counter == 50)
      wb_motor_set_position(motor1, 1);
    else if (counter == 100)
      wb_motor_set_position(motor1, -1);
    else if (counter == 200)
      wb_motor_set_position(motor1, 0);
    else if (counter == 300)
      wb_motor_set_position(motor2, 1);
    else if (counter == 400)
      wb_motor_set_position(motor2, -1);
    else if (counter == 500)
      wb_motor_set_position(motor2, 0);
    else if (counter == 600)
      wb_motor_set_position(motor3, 1);
    else if (counter == 700)
      wb_motor_set_position(motor3, -1);
    else if (counter == 800)
      wb_motor_set_position(motor3, 0);
    else if (counter > 900) {
      wb_motor_set_position(motor1, cos(0.01 * counter));
      wb_motor_set_position(motor2, sin(0.012 * counter));
      wb_motor_set_position(motor3, cos(0.013 * counter));
    }
  }

  wb_robot_cleanup();
  return 0;
}
