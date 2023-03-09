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
 * Description:  Alternate tripod gait using linear motors
 * Author:       Yvan Bourquin
 */

#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 16
#define NUM_MOTORS 12
#define NUM_STATES 6

#define FRONT +0.7
#define BACK -0.7
#define HI +0.02
#define LO -0.02

int main() {
  const char *MOTOR_NAMES[NUM_MOTORS] = {"hip_motor_l0",  "hip_motor_l1",  "hip_motor_l2",  "hip_motor_r0",
                                         "hip_motor_r1",  "hip_motor_r2",  "knee_motor_l0", "knee_motor_l1",
                                         "knee_motor_l2", "knee_motor_r0", "knee_motor_r1", "knee_motor_r2"};
  WbDeviceTag motors[NUM_MOTORS];
  const double pos[NUM_STATES][NUM_MOTORS] = {{BACK, FRONT, BACK, -FRONT, -BACK, -FRONT, LO, HI, LO, HI, LO, HI},
                                              {BACK, FRONT, BACK, -FRONT, -BACK, -FRONT, HI, HI, HI, HI, HI, HI},
                                              {BACK, FRONT, BACK, -FRONT, -BACK, -FRONT, HI, LO, HI, LO, HI, LO},
                                              {FRONT, BACK, FRONT, -BACK, -FRONT, -BACK, HI, LO, HI, LO, HI, LO},
                                              {FRONT, BACK, FRONT, -BACK, -FRONT, -BACK, HI, HI, HI, HI, HI, HI},
                                              {FRONT, BACK, FRONT, -BACK, -FRONT, -BACK, LO, HI, LO, HI, LO, HI}};
  int elapsed = 0;
  int i;

  wb_robot_init();

  for (i = 0; i < NUM_MOTORS; i++) {
    motors[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    if (!motors[i])
      printf("could not find motor: %s\n", MOTOR_NAMES[i]);
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    elapsed++;
    const int state = (elapsed / 25 + 1) % NUM_STATES;
    for (i = 0; i < NUM_MOTORS; i++)
      wb_motor_set_position(motors[i], pos[state][i]);
  }

  wb_robot_cleanup();

  return 0;
}
