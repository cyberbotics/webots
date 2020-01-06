/*
 * Copyright 1996-2020 Cyberbotics Ltd.
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

/*
 * Description:   Implement the functions defined in gripper.h
 */

#include "gripper.h"

#include <webots/motor.h>
#include <webots/robot.h>

#include "tiny_math.h"

#define LEFT 0
#define RIGHT 1

#define MIN_POS 0.0
#define MAX_POS 0.025
#define OFFSET_WHEN_LOCKED 0.021

static WbDeviceTag fingers[2];

void gripper_init() {
  fingers[LEFT] = wb_robot_get_device("finger1");
  fingers[RIGHT] = wb_robot_get_device("finger2");

  wb_motor_set_velocity(fingers[LEFT], 0.03);
  wb_motor_set_velocity(fingers[RIGHT], 0.03);
}

void gripper_grip() {
  wb_motor_set_position(fingers[LEFT], MIN_POS);
  wb_motor_set_position(fingers[RIGHT], MIN_POS);
}

void gripper_release() {
  wb_motor_set_position(fingers[LEFT], MAX_POS);
  wb_motor_set_position(fingers[RIGHT], MAX_POS);
}

void gripper_set_gap(double gap) {
  double v = bound(0.5 * (gap - OFFSET_WHEN_LOCKED), MIN_POS, MAX_POS);
  wb_motor_set_position(fingers[LEFT], v);
  wb_motor_set_position(fingers[RIGHT], v);
}
