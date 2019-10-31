/*
 * Copyright 1996-2019 Cyberbotics Ltd.
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

#include <math.h>
#include <stdio.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 8
#define MAX_SPEED 7.0

enum XYZAComponents { X, Y, Z, ALPHA };
enum Sides { LEFT, RIGHT };

typedef struct _Vector {
  double u;
  double v;
} Vector;

static WbDeviceTag motors[2];

static bool autopilot     = true;
static bool old_autopilot = true;
static int  old_key = -1;


// set left and right motor speed [rad/s]
static void robot_set_speed(double left, double right) {
  wb_motor_set_velocity(motors[0], left);
  wb_motor_set_velocity(motors[1], right);
}

static void check_keyboard() {
  double speeds[2] = {0.0, 0.0};

  int key = wb_keyboard_get_key();
  if (key >= 0) {
    switch (key) {
      case WB_KEYBOARD_UP:
        speeds[LEFT]  = MAX_SPEED;
        speeds[RIGHT] = MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_DOWN:
        speeds[LEFT]  = -MAX_SPEED;
        speeds[RIGHT] = -MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_RIGHT:
        speeds[LEFT]  =  MAX_SPEED;
        speeds[RIGHT] = -MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_LEFT:
        speeds[LEFT]  = -MAX_SPEED;
        speeds[RIGHT] =  MAX_SPEED;
        autopilot = false;
        break;
      case 'A':
        if (key != old_key)  // perform this action just once
          autopilot = !autopilot;
        break;
    }
  }
  if (autopilot != old_autopilot) {
    old_autopilot = autopilot;
    if (autopilot)
      printf("auto control\n");
    else
      printf("manual control\n");
  }

  robot_set_speed(speeds[LEFT], speeds[RIGHT]);
  old_key = key;
}

// autopilot
// Go straight forward
static void run_autopilot() {
  // set the motor speeds
  robot_set_speed(MAX_SPEED, MAX_SPEED);
}

int main(int argc, char *argv[]) {
  // initialize webots communication
  wb_robot_init();

  // print user instructions
  printf("\f");
  printf("You can drive this robot:\n");
  printf("Select the 3D window and use cursor keys:\n");
  printf("Press 'A' to return to the autopilot mode\n");
  printf("\n");

  wb_robot_step(1000);

  const char *names[2] = {"wheel_left_joint", "wheel_right_joint"};

  // get motor tags
  motors[0] = wb_robot_get_device(names[0]); // left
  motors[1] = wb_robot_get_device(names[1]); // right
  wb_motor_set_position(motors[0], INFINITY);
  wb_motor_set_position(motors[1], INFINITY);

  // enable keyboard
  wb_keyboard_enable(TIME_STEP);

  // start forward motion
  robot_set_speed(MAX_SPEED/2.0, MAX_SPEED/2.0);

  // main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    check_keyboard();
    if (autopilot)
      run_autopilot();
  }

  wb_robot_cleanup();

  return 0;
}
