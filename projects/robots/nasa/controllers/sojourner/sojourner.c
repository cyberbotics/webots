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
 * Description:  A controller for the Sojourner robot developped for an EiVd
 *               diploma project of Nicolas Uebelhart.
 */

#include <stdio.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 64
#define VELOCITY 0.6

enum {
  back_left_bogie,
  front_left_bogie,
  front_left_arm,
  back_left_arm,
  front_left_wheel,
  middle_left_wheel,
  back_left_wheel,
  back_right_bogie,
  front_right_bogie,
  front_right_arm,
  back_right_arm,
  front_right_wheel,
  middle_right_wheel,
  back_right_wheel,
  JOINTS_MAX
};

WbDeviceTag joints[JOINTS_MAX];

void move_4_wheels(double v) {
  wb_motor_set_velocity(joints[front_left_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[middle_left_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[back_left_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[front_right_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[middle_right_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[back_right_wheel], v * VELOCITY);

  wb_motor_set_available_torque(joints[middle_right_wheel], 0.0);
  wb_motor_set_available_torque(joints[middle_left_wheel], 0.0);
}

void move_6_wheels(double v) {
  wb_motor_set_available_torque(joints[middle_right_wheel], 2.0);
  wb_motor_set_available_torque(joints[middle_left_wheel], 2.0);

  wb_motor_set_velocity(joints[front_left_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[middle_left_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[back_left_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[front_right_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[middle_right_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[back_right_wheel], v * VELOCITY);
}

void turn_wheels_right() {
  wb_motor_set_position(joints[front_left_arm], 0.4);
  wb_motor_set_position(joints[front_right_arm], 0.227);
  wb_motor_set_position(joints[back_right_arm], -0.227);
  wb_motor_set_position(joints[back_left_arm], -0.4);
}

void turn_wheels_left() {
  wb_motor_set_position(joints[front_left_arm], -0.227);
  wb_motor_set_position(joints[front_right_arm], -0.4);
  wb_motor_set_position(joints[back_right_arm], 0.4);
  wb_motor_set_position(joints[back_left_arm], 0.227);
}

void wheels_straight() {
  wb_motor_set_position(joints[front_left_arm], 0.0);
  wb_motor_set_position(joints[front_right_arm], 0.0);
  wb_motor_set_position(joints[back_right_arm], 0.0);
  wb_motor_set_position(joints[back_left_arm], 0.0);
}

void turn_around(double v) {
  wb_motor_set_position(joints[front_left_arm], -0.87);
  wb_motor_set_position(joints[front_right_arm], 0.87);
  wb_motor_set_position(joints[back_right_arm], -0.87);
  wb_motor_set_position(joints[back_left_arm], 0.87);

  wb_motor_set_velocity(joints[front_left_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[middle_left_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[back_left_wheel], v * VELOCITY);
  wb_motor_set_velocity(joints[front_right_wheel], -v * VELOCITY);
  wb_motor_set_velocity(joints[middle_right_wheel], -v * VELOCITY);
  wb_motor_set_velocity(joints[back_right_wheel], -v * VELOCITY);

  wb_motor_set_available_torque(joints[middle_right_wheel], 0.0);
  wb_motor_set_available_torque(joints[middle_left_wheel], 0.0);
}

int main() {
  // Required to initialize Webots
  wb_robot_init();

  joints[back_left_bogie] = wb_robot_get_device("BackLeftBogie");
  joints[front_left_bogie] = wb_robot_get_device("FrontLeftBogie");
  joints[front_left_arm] = wb_robot_get_device("FrontLeftArm");
  joints[back_left_arm] = wb_robot_get_device("BackLeftArm");
  joints[front_left_wheel] = wb_robot_get_device("FrontLeftWheel");
  joints[middle_left_wheel] = wb_robot_get_device("MiddleLeftWheel");
  joints[back_left_wheel] = wb_robot_get_device("BackLeftWheel");
  joints[back_right_bogie] = wb_robot_get_device("BackRightBogie");
  joints[front_right_bogie] = wb_robot_get_device("FrontRightBogie");
  joints[front_right_arm] = wb_robot_get_device("FrontRightArm");
  joints[back_right_arm] = wb_robot_get_device("BackRightArm");
  joints[front_right_wheel] = wb_robot_get_device("FrontRightWheel");
  joints[middle_right_wheel] = wb_robot_get_device("MiddleRightWheel");
  joints[back_right_wheel] = wb_robot_get_device("BackRightWheel");

  wb_motor_set_position(joints[front_left_wheel], INFINITY);
  wb_motor_set_position(joints[middle_left_wheel], INFINITY);
  wb_motor_set_position(joints[back_left_wheel], INFINITY);
  wb_motor_set_position(joints[front_right_wheel], INFINITY);
  wb_motor_set_position(joints[middle_right_wheel], INFINITY);
  wb_motor_set_position(joints[back_right_wheel], INFINITY);

  printf("Select the 3D window and use the keyboard to drive this robot:\n");
  printf("\n");
  printf("Q: forwards-left;  W: forwards;  E: forwards-right\n");
  printf("S: spin counter-clockwise\n");
  printf("Y: backwards-left; X: backwards; C: backwards-right\n");
  wb_keyboard_enable(TIME_STEP);

  // start moving
  move_6_wheels(1.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    int key = wb_keyboard_get_key();
    switch (key) {
      case 'W':
        // forwards
        wheels_straight();
        move_6_wheels(1.0);
        break;
      case 'X':
        // backwards
        wheels_straight();
        move_6_wheels(-1.0);
        break;
      case 'Q':
        // forwards left
        turn_wheels_left();
        move_4_wheels(1.0);
        break;
      case 'E':
        // forwards right
        turn_wheels_right();
        move_4_wheels(1.0);
        break;
      case 'Y':
        // backwards left
        turn_wheels_left();
        move_4_wheels(-1.0);
        break;
      case 'C':
        // backwards right
        turn_wheels_right();
        move_4_wheels(-1.0);
        break;
      case 'S':
        // spin counter-clockwise
        turn_around(1.0);
        break;
    }
  }

  wb_robot_cleanup();

  return 0;
}
