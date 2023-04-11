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

/*-----------------------------------------------------------------------------
  Description:  Running quadruped with active hip joints and
                active/passive (spring & damper) knee joints
-----------------------------------------------------------------------------*/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

// control step duration in milliseconds
#define TIME_STEP 16

enum { RUN, STOP };

int main() {
  WbDeviceTag motors[6];
  const char *MOTOR_NAMES[] = {"hip0", "hip1", "hip2", "hip3", "spine", NULL};
  const double freq = 1.7;   // motion frequency
  double ampl = 1.0;         // motion amplitude
  double spine_angle = 0.0;  // spine angle
  double t = 0.0;            // elapsed simulation time
  int mode = RUN;
  int prev_key = 0;

  wb_robot_init();

  // hip and spine motor names
  // we don't control the knee motor: knees are passive

  // get motor device tags
  int i;
  for (i = 0; MOTOR_NAMES[i]; i++) {
    motors[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    assert(motors[i]);
  }

  // enable keyboard
  wb_keyboard_enable(TIME_STEP);

  // print user instructions
  printf("You can drive me!\n");
  printf("Select the 3D window and press:\n");
  printf("'Left' and 'Right' cursor keys to TURN\n");
  printf("'Up' key to ACCELERATE\n");
  printf("'Down' key to DECELERATE\n");
  printf("'S' key to STOP the robot\n");
  printf("'R' key to RUN the robot\n");
  while (wb_robot_step(TIME_STEP) != -1) {
    const int new_key = wb_keyboard_get_key();
    if (new_key != prev_key) {
      switch (new_key) {
        case WB_KEYBOARD_LEFT:
          // change spine angle
          if (spine_angle > -0.6) {
            spine_angle -= 0.1;
            wb_motor_set_position(motors[4], spine_angle);
          }
          break;
        case WB_KEYBOARD_RIGHT:
          // change spine angle
          if (spine_angle < 0.6) {
            spine_angle += 0.1;
            wb_motor_set_position(motors[4], spine_angle);
          }
          break;
        case WB_KEYBOARD_UP:
          // increase amplitude
          if (ampl < 1.7)
            ampl += 0.1;
          printf("Motion amplitude: %f\n", ampl);
          break;
        case WB_KEYBOARD_DOWN:
          // decrease amplitude
          if (ampl > -1.7)
            ampl -= 0.1;
          printf("Motion amplitude: %f\n", ampl);
          break;
        case 'S':
          mode = STOP;
          printf("Stopped\n");
          break;
        case 'R':
          mode = RUN;
          printf("Running\n");
          break;
      }
      prev_key = new_key;
    }

    if (mode == RUN) {
      // compute front and hind legs signals
      double phase = t * 2.0 * M_PI * freq;
      double fpos = ampl * 0.5 * sin(phase) + 0.1;
      double hpos = ampl * 0.8 * sin(phase + 2.0) - 0.2;

      // actuate front legs
      wb_motor_set_position(motors[0], fpos);
      wb_motor_set_position(motors[2], fpos);

      // actuate hind legs
      wb_motor_set_position(motors[1], hpos);
      wb_motor_set_position(motors[3], hpos);
    }

    // adjust elapsed time
    t += (double)TIME_STEP / 1000.0;
  }

  wb_robot_cleanup();

  return 0;
}
