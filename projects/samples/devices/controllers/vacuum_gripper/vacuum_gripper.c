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
   Description: Example demonstrating the use of the VacuumGripper device to simulate a vacuum suction gripper.
*/

#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/vacuum_gripper.h>

#include <stdio.h>
#include <stdlib.h>

#define NB_STEPS 20

static int time_step = 32;

static void wait_until_target_position(WbDeviceTag inertial_unit, const double roll_target, const double pitch_target,
                                       const double yaw_target, const int maxSteps) {
  int steps = maxSteps;
  while (maxSteps < 0 || steps > 0) {
    // execute a simulation step
    if (wb_robot_step(time_step) == -1)
      break;

    // read inertial unit values
    const double *ground_truth_attitude = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);

    // see if target position was reached
    if (fabs(ground_truth_attitude[0] - roll_target) < 0.01 && fabs(ground_truth_attitude[1] - pitch_target) < 0.01 &&
        fabs(ground_truth_attitude[2] - yaw_target) < 0.01)
      break;

    if (maxSteps >= 0)
      --steps;
  }
}

int main(int argc, const char *argv[]) {
  // initialize webots API
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  WbDeviceTag vacuum_gripper = wb_robot_get_device("vacuum gripper");

  // move the robotic arm to random target positions
  WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, time_step);
  WbDeviceTag roll_motor = wb_robot_get_device("roll motor");
  WbDeviceTag pitch_motor = wb_robot_get_device("pitch motor");
  WbDeviceTag yaw_motor = wb_robot_get_device("yaw motor");

  // start moving arm to target
  wb_motor_set_position(roll_motor, 0);
  wb_motor_set_position(pitch_motor, 0);
  wb_motor_set_position(yaw_motor, 0);
  wait_until_target_position(inertial_unit, 0, 0, 0, -1);

  printf("Turn on vacuum gripper\n");
  wb_vacuum_gripper_turn_on(vacuum_gripper);

  wb_vacuum_gripper_enable_presence(vacuum_gripper, time_step);

  bool connected = false;
  for (int i = 0; i < NB_STEPS; i++) {
    if (wb_vacuum_gripper_get_presence(vacuum_gripper) != connected) {
      connected = wb_vacuum_gripper_get_presence(vacuum_gripper);
      printf("Object has been %s", connected ? "picked" : "released");
    }

    // choose a random target (based on seed)
    const double yaw_target = i == 0 ? 0 : rand() / (double)RAND_MAX * 2.0 * M_PI - M_PI;
    const double pitch_target = i == 0 ? 0 : rand() / (double)RAND_MAX * 1.4 - 0.7;
    const double roll_target = i == 0 ? 0 : rand() / (double)RAND_MAX * 2.0 * M_PI - M_PI;

    // start moving arm to target
    wb_motor_set_position(roll_motor, roll_target);
    wb_motor_set_position(pitch_motor, pitch_target);
    wb_motor_set_position(yaw_motor, yaw_target);
    wait_until_target_position(inertial_unit, roll_target, pitch_target, yaw_target, 100);

    if (i == 12) {
      printf("Turn off vacuum gripper\n");
      wb_vacuum_gripper_turn_off(vacuum_gripper);
    }
  }

  wb_vacuum_gripper_disable_presence(vacuum_gripper);

  // cleanup webots resources
  wb_robot_cleanup();

  return 0;
}
