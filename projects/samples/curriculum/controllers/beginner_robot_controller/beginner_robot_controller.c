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

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Global defines
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define SIMULATION 0  // for robot_get_mode() function
#define REALITY 2     // for robot_get_mode() function
#define TIME_STEP 32  // [ms]

// variables specific to the simulation
#define SPEED_UNIT 0.00628
#define MAX_SPEED 200
#define INCR 4

// LEDs
#define NB_LEDS 8
WbDeviceTag led[NB_LEDS];
WbDeviceTag cam;
WbDeviceTag accelerometer;
WbDeviceTag left_motor, right_motor;

// create two variables corresponding to speed of each robots
int speed[2] = {0, 0};

// robot reset function
static void reset(void) {
  // get Leds (from led0 to led7)
  int i;
  char text[5] = "led0";
  for (i = 0; i < NB_LEDS; i++) {
    led[i] = wb_robot_get_device(text);  // get leds
    text[3]++;
  }

  // obtain and enable the camera
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP);

  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, TIME_STEP);

  // get a handler to the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // enable keyboard
  wb_keyboard_enable(TIME_STEP);

  // Switch off leds
  for (i = 0; i < NB_LEDS; i++)
    wb_led_set(led[i], FALSE);
}

static void run(void) {
  int key = wb_keyboard_get_key();

  // do the appropriate thing according to the pressed key
  switch (key) {
    case 'S':
      if (speed[LEFT] < MAX_SPEED)
        speed[LEFT] += INCR;
      break;
    case 'X':
      if (speed[LEFT] > -(MAX_SPEED + 1))
        speed[LEFT] -= INCR;
      break;
    case 'C':
      if (speed[RIGHT] > -(MAX_SPEED + 1))
        speed[RIGHT] -= INCR;
      break;
    case 'D':
      if (speed[RIGHT] < MAX_SPEED)
        speed[RIGHT] += INCR;
      break;
    case WB_KEYBOARD_UP:
      if (speed[LEFT] < MAX_SPEED)
        speed[LEFT] += INCR;
      if (speed[RIGHT] < MAX_SPEED)
        speed[RIGHT] += INCR;
      break;
    case WB_KEYBOARD_DOWN:
      if (speed[LEFT] > -MAX_SPEED)
        speed[LEFT] -= INCR;
      if (speed[RIGHT] > -(MAX_SPEED + 1))
        speed[RIGHT] -= INCR;
      break;
    case WB_KEYBOARD_LEFT:
      if (speed[RIGHT] < MAX_SPEED)
        speed[RIGHT] += INCR;
      if (speed[LEFT] > -(MAX_SPEED + 1))
        speed[LEFT] -= INCR;
      break;
    case WB_KEYBOARD_RIGHT:
      if (speed[LEFT] < MAX_SPEED)
        speed[LEFT] += INCR;
      if (speed[RIGHT] > -(MAX_SPEED + 1))
        speed[RIGHT] -= INCR;
      break;
    case 'R':
      speed[LEFT] = speed[RIGHT] = 0;
      break;
    default:
      break;
  }

  // when robot goes backward, switch on back leds like in big trucks :-)
  if (speed[LEFT] < 0 && speed[RIGHT] < 0)
    wb_led_set(led[4], TRUE);
  else
    wb_led_set(led[4], FALSE);

  // send speed values to motors
  wb_motor_set_velocity(left_motor, SPEED_UNIT * speed[LEFT]);
  wb_motor_set_velocity(right_motor, SPEED_UNIT * speed[RIGHT]);

  return;
}

int main() {
  wb_robot_init(); /* initialize the webots controller library */

  reset();

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1)
    run();

  wb_robot_cleanup();

  return 0;
}
