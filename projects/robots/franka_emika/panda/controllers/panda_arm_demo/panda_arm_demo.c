/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
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
#define OPEN_HAND 0
#define CLOSE_HAND 1

static float block_positions[3][3] = {{0.37, -2.7, 2.9}, {0.55, -2.37, 2.75}, {0.74, -1.96, 2.53}};
static float block_clearings[3][3] = {{-0.1, -2.78, 2.61}, {0.22, -2.48, 2.53}, {0.33, -2.18, 2.35}};

enum Joint { JOINT1, JOINT2, JOINT3, JOINT4, JOINT5, JOINT6, JOINT7, FINGER1, FINGER2 };
enum Blocks { BLOCK1, BLOCK2, BLOCK3 };

static WbDeviceTag motors[9];
static WbDeviceTag sensors[9];

void hand_control(int state) {
  if (state == OPEN_HAND) {
    wb_motor_set_position(motors[FINGER1], 0.02);
    wb_motor_set_position(motors[FINGER2], 0.02);
  } else {
    wb_motor_set_position(motors[FINGER1], 0.012);
    wb_motor_set_position(motors[FINGER2], 0.012);
  }

  wb_robot_step(TIME_STEP * 20);
}

void move_to_block(int block) {
  wb_motor_set_position(motors[JOINT2], block_positions[block][0]);
  wb_motor_set_position(motors[JOINT4], block_positions[block][1]);
  wb_motor_set_position(motors[JOINT6], block_positions[block][2]);

  wb_robot_step(TIME_STEP * 30);
}

void move_to_clearing(int block) {
  wb_motor_set_position(motors[JOINT2], block_clearings[block][0]);
  wb_motor_set_position(motors[JOINT4], block_clearings[block][1]);
  wb_motor_set_position(motors[JOINT6], block_clearings[block][2]);

  wb_robot_step(TIME_STEP * 30);
}

void sequence(int from, int to) {
  move_to_block(from);

  hand_control(CLOSE_HAND);

  move_to_clearing(from);

  move_to_clearing(to);

  move_to_block(to);

  hand_control(OPEN_HAND);

  move_to_clearing(to);
}

int main(int argc, char **argv) {
  wb_robot_init();

  char device_name[30];

  for (int i = 0; i < 9; ++i) {
    const char *prefix = i < 7 ? "panda_joint" : "panda_finger_joint";
    int offset = i < 7 ? 0 : 7;
    sprintf(device_name, "%s%d", prefix, i + 1 - offset);
    motors[i] = wb_robot_get_device(device_name);
    sprintf(device_name, "%s%d_sensor", prefix, i + 1 - offset);
    sensors[i] = wb_robot_get_device(device_name);
    wb_position_sensor_enable(sensors[i], TIME_STEP);
  }
  /*
  hand_control(OPEN_HAND);

  wb_robot_step(TIME_STEP * 20);

  wb_motor_set_position(motors[JOINT2], 0.55);
  wb_motor_set_position(motors[JOINT4], -2.37);
  wb_motor_set_position(motors[JOINT6], 2.75);

  wb_robot_step(TIME_STEP * 20);

  hand_control(CLOSE_HAND);
  wb_robot_step(TIME_STEP * 20);

  wb_motor_set_position(motors[JOINT2], 0.22);
  wb_motor_set_position(motors[JOINT4], -2.48);
  wb_motor_set_position(motors[JOINT6], 2.53);

  wb_robot_step(TIME_STEP * 20);


  wb_motor_set_position(motors[JOINT2], -0.1);
  wb_motor_set_position(motors[JOINT4], -2.78);
  wb_motor_set_position(motors[JOINT6], 2.61);

  wb_robot_step(TIME_STEP * 20);

  wb_motor_set_position(motors[JOINT2], 0.37);
  wb_motor_set_position(motors[JOINT4], -2.7);
  wb_motor_set_position(motors[JOINT6], 2.90);

  wb_robot_step(TIME_STEP * 20);

  hand_control(OPEN_HAND);
  */
  // wb_motor_set_position(motors[JOINT2], 0.1);

  hand_control(OPEN_HAND);

  for (int i = 0; i < 10; ++i) {
    sequence(BLOCK1, BLOCK3);

    sequence(BLOCK2, BLOCK1);

    sequence(BLOCK3, BLOCK2);
  };

  wb_robot_cleanup();

  return 0;
}
