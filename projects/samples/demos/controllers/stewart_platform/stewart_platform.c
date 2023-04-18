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

//
// Description:  Simple example controller for stewart_platform.wbt world
//               The stewart platform is initiallly loaded with 5 yellow boxes, then the platform
//               performs various movements and the yellow boxes fall on the floor.
//

#include <math.h>
#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 64
#define NUM_PISTONS 6

// linear motors
static WbDeviceTag pistons[NUM_PISTONS];

// global variables
static int i, j;
static double t = 0.0;

// find the devices
static void find_devices() {
  for (i = 0; i < NUM_PISTONS; i++) {
    char name[64];
    sprintf(name, "piston%d", i);
    pistons[i] = wb_robot_get_device(name);
  }
}

// clockwise and counter-clockwise twist of the stewart platform
static void twist() {
  for (j = 0; j < 2; j++) {
    const double AMPL = 0.4;  // meters
    for (i = 0; i < NUM_PISTONS; i++)
      wb_motor_set_position(pistons[i], ((j + i) % 2) ? AMPL : -AMPL);

    for (t = 0; t < 1.0; t += TIME_STEP / 1000.0)
      wb_robot_step(TIME_STEP);
  }
}

// inclination of the stewart platform
static void rotate() {
  for (t = 0; t < 15.0; t += TIME_STEP / 1000.0) {
    const double FREQ = 1.0;  // Hz
    double ampl = 0.35 * t / 15;
    double phase = 2 * M_PI * FREQ * t;
    for (i = 0; i < 3; i++) {
      double phase_shift = i * 2 * M_PI / 3;
      wb_motor_set_position(pistons[2 * i], ampl * sin(phase + phase_shift));
      wb_motor_set_position(pistons[2 * i + 1], ampl * sin(phase + phase_shift));
    }
    wb_robot_step(TIME_STEP);
  }
}

// jumps with full amplitude
static void hop_hop_hop() {
  for (t = 0; t < 10.0; t += TIME_STEP / 1000.0) {
    const double FREQ = 1.2;  // Hz
    const double AMPL = 0.4;
    double phase = 2 * M_PI * FREQ * t;
    for (i = 0; i < NUM_PISTONS; i++)
      wb_motor_set_position(pistons[i], AMPL * sin(phase));

    wb_robot_step(TIME_STEP);
  }
}

// main program
int main() {
  wb_robot_init();  // initialize webots
  find_devices();
  twist();
  rotate();
  hop_hop_hop();
  for (;;)
    wb_robot_step(TIME_STEP);  // wait forever
  return 0;
}
