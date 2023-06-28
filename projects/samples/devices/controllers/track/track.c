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
 * Description:   An example of a controller using the Track node to move
 *                tracked robot.
 *
 *                The Coulomb friction value set in the WorldInfo contactProperties
 *                has a big influence on the behavior of the robot and especially
 *                on the turning motion.
 */

#include <webots/motor.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag leftMotor = wb_robot_get_device("left motor");
  WbDeviceTag rightMotor = wb_robot_get_device("right motor");
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);
  int timeStep = wb_robot_get_basic_time_step();

  // go straight
  wb_motor_set_velocity(leftMotor, 0.1);
  wb_motor_set_velocity(rightMotor, 0.1);

  int i = 1400;
  while (wb_robot_step(timeStep) != -1 && i > 0) {
    --i;
  };

  // turn left
  wb_motor_set_velocity(leftMotor, -0.1);
  wb_motor_set_velocity(rightMotor, 0.1);

  i = 1000;
  while (wb_robot_step(timeStep) != -1 && i > 0) {
    --i;
  };

  // turn right
  wb_motor_set_velocity(leftMotor, 0.1);
  wb_motor_set_velocity(rightMotor, -0.1);

  while (wb_robot_step(timeStep) != -1) {
  };

  wb_robot_cleanup();

  return 0;
}
