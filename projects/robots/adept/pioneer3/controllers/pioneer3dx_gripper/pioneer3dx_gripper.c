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
 * Description:  A controller moving the Pioneer3DX and its gripper.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define GRIPPER_MOTOR_MAX_SPEED 0.1

static WbDeviceTag wheel_motors[3];
static WbDeviceTag gripper_motors[2];
static int time_step = 0;

static void initialize() {
  /* necessary to initialize Webots */
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  gripper_motors[0] = wb_robot_get_device("lift motor");
  gripper_motors[1] = wb_robot_get_device("finger motor::left");
  wheel_motors[0] = wb_robot_get_device("left wheel");
  wheel_motors[1] = wb_robot_get_device("right wheel");
  // Specify velocity control mode
  wb_motor_set_position(wheel_motors[0], INFINITY);
  wb_motor_set_position(wheel_motors[1], INFINITY);
  wb_motor_set_velocity(wheel_motors[0], 0.0);
  wb_motor_set_velocity(wheel_motors[1], 0.0);
}

void step(double seconds) {
  const double ms = seconds * 1000.0;
  int elapsed_time = 0;
  while (elapsed_time < ms) {
    wb_robot_step(time_step);
    elapsed_time += time_step;
  }
}

void lift(double position) {
  wb_motor_set_velocity(gripper_motors[0], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[0], position);
}

void moveFingers(double position) {
  wb_motor_set_velocity(gripper_motors[1], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[1], position);
}

void moveForwards(double speed) {
  wb_motor_set_velocity(wheel_motors[0], speed);
  wb_motor_set_velocity(wheel_motors[1], speed);
}

void turn(double speed) {
  wb_motor_set_velocity(wheel_motors[0], speed);
  wb_motor_set_velocity(wheel_motors[1], -speed);
}

void stop(double seconds) {
  wb_motor_set_velocity(wheel_motors[0], 0.0);
  wb_motor_set_velocity(wheel_motors[1], 0.0);
  step(seconds);
}

int main() {
  initialize();
  lift(0.05);
  moveFingers(0.06);
  moveForwards(1.25);
  step(2.0);
  stop(0.5);
  moveFingers(0.01);
  step(0.5);
  lift(0.0);
  step(0.5);
  turn(2.0);
  step(1.0);
  moveForwards(1.25);
  step(5.5);
  turn(-1.5);
  step(1.5);
  stop(0.5);
  lift(0.05);
  step(0.5);
  moveFingers(0.04);
  step(0.5);
  stop(0.5);
  return 0;
}
