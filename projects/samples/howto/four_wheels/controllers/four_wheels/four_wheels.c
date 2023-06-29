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
 * Description:  Four wheel vehicle examples
 */

#include <string.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 16

// motor wheels
static WbDeviceTag wheels[4];

// only for Ackerman:
static WbDeviceTag steering[2];
static int has_steering = 0;

static void wheel_set_speed(WbDeviceTag tag, double speed) {
  if (speed >= 0.0) {
    wb_motor_set_position(tag, INFINITY);
    wb_motor_set_velocity(tag, speed);
  } else {
    wb_motor_set_position(tag, -INFINITY);
    wb_motor_set_velocity(tag, -speed);
  }
}

static void wheels_set_speed(double speed) {
  int i;
  for (i = 0; i < 4; i++)
    wheel_set_speed(wheels[i], speed);
}

int main(int argc, char **argv) {
  // initialize webots
  wb_robot_init();

  // only for Ackerman
  if (strcmp(wb_robot_get_name(), "ackerman") == 0) {
    has_steering = 1;
    steering[0] = wb_robot_get_device("left_steer");
    steering[1] = wb_robot_get_device("right_steer");
  }

  // all vehicles
  wheels[0] = wb_robot_get_device("rear_right_wheel");
  wheels[1] = wb_robot_get_device("rear_left_wheel");
  wheels[2] = wb_robot_get_device("front_right_wheel");
  wheels[3] = wb_robot_get_device("front_left_wheel");

  // go forward
  wheels_set_speed(2.0);

  // forever
  double time = 0.0;
  while (wb_robot_step(TIME_STEP) != -1) {
    time += TIME_STEP / 1000.0;

    // only for Ackerman
    if (has_steering) {
      double dir = 0.5 * sin(time);
      wb_motor_set_position(steering[0], dir);
      wb_motor_set_position(steering[1], dir);
    }
  }

  wb_robot_cleanup();

  return 0;
}
