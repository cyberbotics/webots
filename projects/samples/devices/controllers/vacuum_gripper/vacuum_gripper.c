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

#define NB_STEPS 20
#define TARGETS_COUNT 10

static int time_step = 32;
static double TARGET_ANGLES[TARGETS_COUNT][3] = {{2.137462, -0.147864, 1.778765},   {1.875154, 0.576306, -1.900341},
                                                 {-1.035326, 0.375521, -1.396283},  {0.339103, -0.031644, 0.809720},
                                                 {-1.035326, 0.375521, -1.396283},  {2.615031, 0.189996, 1.365317},
                                                 {-2.157149, -0.138678, -2.326095}, {0.339103, -0.031644, 0.809720},
                                                 {1.875154, 0.576306, -1.900341},   {-1.035326, -0.375521, -1.396283}};

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
  WbDeviceTag gripper_motor = wb_robot_get_device("gripper motor");

  // move arm to pick position
  wb_motor_set_position(roll_motor, 0);
  wb_motor_set_position(pitch_motor, 0);
  wb_motor_set_position(yaw_motor, 0);
  wait_until_target_position(inertial_unit, 0, 0, 0, -1);

  printf("Turn on vacuum gripper\n");
  wb_vacuum_gripper_turn_on(vacuum_gripper);
  wb_vacuum_gripper_enable_presence(vacuum_gripper, time_step);

  double slider_position = 0.0;
  while (wb_robot_step(time_step) != -1) {
    if (wb_vacuum_gripper_get_presence(vacuum_gripper))
      break;

    slider_position += 0.001;
    wb_motor_set_position(gripper_motor, slider_position);
  }

  wb_robot_step(time_step);
  wb_motor_set_position(gripper_motor, 0.0);
  wb_robot_step(10 * time_step);

  bool connected = false;
  for (int i = 0; i < TARGETS_COUNT; i++) {
    if (wb_vacuum_gripper_get_presence(vacuum_gripper) != connected) {
      connected = wb_vacuum_gripper_get_presence(vacuum_gripper);
      printf("Object has been %s\n", connected ? "picked" : "released");
    }

    // move arm to target positions
    wb_motor_set_position(roll_motor, TARGET_ANGLES[i][2]);
    wb_motor_set_position(pitch_motor, TARGET_ANGLES[i][1]);
    wb_motor_set_position(yaw_motor, TARGET_ANGLES[i][0]);
    wait_until_target_position(inertial_unit, TARGET_ANGLES[i][2], TARGET_ANGLES[i][1], TARGET_ANGLES[i][0], 100);

    if (i == 8) {
      printf("Turn off vacuum gripper\n");
      wb_vacuum_gripper_turn_off(vacuum_gripper);
    }
  }

  wb_vacuum_gripper_disable_presence(vacuum_gripper);

  wb_robot_cleanup();

  return 0;
}
