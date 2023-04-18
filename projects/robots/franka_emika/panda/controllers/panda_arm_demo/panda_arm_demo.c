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

#include <stdio.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 32

static float block_positions[3][3] = {{0.37, -2.7, 2.9}, {0.52, -2.37, 2.73}, {0.74, -1.96, 2.53}};
static float block_clearings[3][3] = {{-0.1, -2.78, 2.61}, {0.22, -2.48, 2.53}, {0.33, -2.18, 2.35}};

enum Joints { JOINT1, JOINT2, JOINT3, JOINT4, JOINT5, JOINT6, JOINT7, FINGER };
enum Blocks { BLOCK1, BLOCK2, BLOCK3 };
enum HandCommands { OPEN_HAND, CLOSE_HAND };

static WbDeviceTag motors[9];

void hand_control(int command) {
  // based on the state, opens or closes the gripper
  wb_motor_set_position(motors[FINGER], (command == OPEN_HAND) ? 0.02 : 0.012);

  // delay for the action to take place
  wb_robot_step(TIME_STEP * 10);
}

void move_to_block(int block) {
  wb_motor_set_position(motors[JOINT2], block_positions[block][0]);
  wb_motor_set_position(motors[JOINT4], block_positions[block][1]);
  wb_motor_set_position(motors[JOINT6], block_positions[block][2]);
  // delay for the movement to take place
  wb_robot_step(TIME_STEP * 20);
}

void move_to_clearing(int block) {
  wb_motor_set_position(motors[JOINT2], block_clearings[block][0]);
  wb_motor_set_position(motors[JOINT4], block_clearings[block][1]);
  wb_motor_set_position(motors[JOINT6], block_clearings[block][2]);
  // delay for the movement to take place
  wb_robot_step(TIME_STEP * 20);
}

void sequence(int origin_block, int target_block) {
  // executes a sequence of actions that moves the origin block to the target block position
  move_to_block(origin_block);

  hand_control(CLOSE_HAND);

  move_to_clearing(origin_block);

  move_to_clearing(target_block);

  move_to_block(target_block);

  hand_control(OPEN_HAND);

  move_to_clearing(target_block);
}

int main(int argc, char **argv) {
  wb_robot_init();

  char device_name[30];
  // retrive the motor references of all the joints
  for (int i = 0; i < 7; ++i) {
    const char *prefix = "panda_joint";
    sprintf(device_name, "%s%d", prefix, i + 1);
    motors[i] = wb_robot_get_device(device_name);
  }

  motors[7] = wb_robot_get_device("panda_finger::right");
  hand_control(OPEN_HAND);

  for (int i = 0; i < 10; ++i) {
    sequence(BLOCK1, BLOCK3);
    sequence(BLOCK2, BLOCK1);
    sequence(BLOCK3, BLOCK2);
  };

  wb_robot_cleanup();

  return 0;
}
