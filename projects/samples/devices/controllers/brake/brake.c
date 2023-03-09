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
 * Description: Wait 4 seconds before to slow down the motor linked with the wheel using the Brake node.
 */

#include <webots/brake.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <stdio.h>

int main(int argc, char **argv) {
  wb_robot_init();
  int time_step = (int)wb_robot_get_basic_time_step();

  // Get the brake device.
  WbDeviceTag brake = wb_robot_get_device("brake");

  // Get the motor linked with the wheel and apply a torque.
  WbDeviceTag motor = wb_robot_get_device("motor");
  wb_motor_set_torque(motor, 0.1);

  // Get the linear motors linked with the red blocks.
  // They are present only for the purpose of improving the understanding of the scene.
  WbDeviceTag linear_motor_a = wb_robot_get_device("linear motor a");
  WbDeviceTag linear_motor_b = wb_robot_get_device("linear motor b");

  // Display the welcome message.
  printf("Start the wheel motor...\n");

  while (wb_robot_step(time_step) != -1) {
    if (wb_robot_get_time() == 4.0) {
      // At four seconds, the movement of the linear motors is started.
      printf("Brake activated!\n");
      wb_motor_set_position(linear_motor_a, wb_motor_get_max_position(linear_motor_a));
      wb_motor_set_position(linear_motor_b, wb_motor_get_max_position(linear_motor_b));
    } else if (wb_robot_get_time() > 4.13) {
      // When the red blocks touch the wheel, the braking of the main engine is performed.
      wb_brake_set_damping_constant(brake, 1.0);
      wb_motor_set_torque(motor, 0.0);
      break;
    }
  }

  // Wait until the simulation is done.
  while (wb_robot_step(time_step) != -1) {
  }

  wb_robot_cleanup();

  return 0;
}
