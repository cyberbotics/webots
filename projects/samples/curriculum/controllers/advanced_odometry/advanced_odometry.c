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
 * Description:   Advanced exercise on odometry
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../lib/odometry.h"
#include "../../lib/odometry_goto.h"

// Global defines
#define PI 3.14159265358979
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define SIMULATION 0  // for robot_get_mode() function
#define REALITY 2     // for robot_get_mode() function
#define TIME_STEP 64  // [ms]

// variables specific to the simulation
#define VERBOSE 1
#define INCR 10
#define MIN_SPEED 20
#define MAX_SPEED 200
#define INCREMENT_TEST 1000
#define RATIO_TEST 14133
#define SCALING_TEST 0.3

// wheel
#define SPEED_UNIT 0.00628
#define ENCODER_UNIT 159.23
static double left_encoder_offset = 0.0;
static double right_encoder_offset = 0.0;
WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;

// create two variables corresponding to speed of each robots
int speed[2] = {0, 0};
int pspeed[2] = {0, 0};
int cpt = 0;

// Instantiate odometry track and goto structures
struct sOdometryTrack ot;
struct sOdometryGoto og;

static void init(void);
static void print_position(void);
static void set_speed(int l, int r);
static void goto_position(float x, float y, float theta);
static void test_increment(int val);
static void test_ratio(int val);
static void test_diameter(void);
static void test_scaling(void);
static void step();

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void init(void) {
  // robot init speed is 0
  speed[LEFT] = speed[RIGHT] = 0;
  pspeed[LEFT] = pspeed[RIGHT] = 0;
  wb_motor_set_velocity(left_motor, SPEED_UNIT * speed[LEFT]);
  wb_motor_set_velocity(right_motor, SPEED_UNIT * speed[RIGHT]);

  printf("Init OK\n");
  return;
}

/*
 * Prints the robot's current position
 */
static void print_position(void) {
  printf("Current position : (%f,%f,%f)\n", ot.result.x, ot.result.y, ot.result.theta);

  return;
}

/*
 * This functions sets the speed of both motors sending the command if and only if
 * the speed has changed.
 */
static void set_speed(int l, int r) {
  pspeed[LEFT] = speed[LEFT];
  pspeed[RIGHT] = speed[RIGHT];
  speed[LEFT] = l;
  speed[RIGHT] = r;

  if (pspeed[LEFT] != speed[LEFT] || pspeed[RIGHT] != speed[RIGHT]) {
    wb_motor_set_velocity(left_motor, SPEED_UNIT * speed[LEFT]);
    wb_motor_set_velocity(right_motor, SPEED_UNIT * speed[RIGHT]);
  }
}

/*
 * Send the robot to a relative position (x,y)
 * using the "odometry" and "odometry_goto" modules
 */
static void goto_position(float x, float y, float theta) {
  if (VERBOSE > 0)
    printf("Going to (%f, %f, %f)\n", x, y, theta);

  // Set a target position
  odometry_goto_set_goal(&og, x, y, theta);

  // Move until the robot is close enough to the target position
  while (og.result.atgoal == 0) {
    // Update position and calculate new speeds
    odometry_track_step_pos(og.track, ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset),
                            ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset));
    odometry_goto_step(&og);

    // Set the wheel speed
    set_speed(og.result.speed_left, og.result.speed_right);

    if (VERBOSE > 1) {
      print_position();
    }

    step();
  }

  if (VERBOSE > 0)
    print_position();

  return;
}

static void test_increment(int val) {
  init();

  // linear speed increase over 100 increments
  while (ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) < 100 ||
         ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset) < 100) {
    int s = (int)(MAX_SPEED *
                  (ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) +
                   ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset)) /
                  200);
    s = s < MIN_SPEED ? MIN_SPEED : s;
    s = s > MAX_SPEED ? MAX_SPEED : s;
    set_speed(s, s);
    step();
  }

  // max speed
  while (ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) < val - 400 &&
         ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset) < val - 400) {
    set_speed(MAX_SPEED, MAX_SPEED);
    step();
  }

  // linear speed decrease over 400 increments
  while (ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) < val ||
         ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset) < val) {
    int s = (int)(MAX_SPEED *
                  (val - (ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) +
                          ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset)) /
                           2) /
                  400);
    s = s < MIN_SPEED ? MIN_SPEED : s;
    s = s > MAX_SPEED ? MAX_SPEED : s;
    set_speed(s, s);
    step();
  }

  printf("INCREMENT test finished\n");
  init();
}

static void test_ratio(int val) {
  init();

  // linear speed increase over 100 increments
  while (ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) > -100 ||
         ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset) < 100) {
    int s = (int)(MAX_SPEED *
                  (-ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) +
                   ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset)) /
                  200);
    s = s < MIN_SPEED ? MIN_SPEED : s;
    s = s > MAX_SPEED ? MAX_SPEED : s;
    set_speed(-s, s);
    step();
  }

  while (ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) > -val + 400 &&
         ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset) < val - 400) {
    set_speed(-MAX_SPEED, MAX_SPEED);
    step();
  }

  // linear speed decrease over 400 increments
  while (ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) > -val ||
         ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset) < val) {
    int s = (int)(MAX_SPEED *
                  (val - (-ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) +
                          ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset)) /
                           2) /
                  400);
    s = s < MIN_SPEED ? MIN_SPEED : s;
    s = s > MAX_SPEED ? MAX_SPEED : s;
    set_speed(-s, s);
    step();
  }

  printf("RATIO test finished\n");
  init();
}

static void test_diameter(void) {
  init();

  goto_position(0, 0.2, PI);
  goto_position(-0.2, 0.2, -PI / 2);
  goto_position(-0.2, 0, 0);
  goto_position(0, 0, PI / 2);

  printf("DIAMETER test finished\n");

  init();

  return;
}

static void test_scaling(void) {
  init();
  odometry_track_step_pos(&ot, ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset),
                          ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset));

  // linear speed increase over 100 increments
  while (ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) < 100 ||
         ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset) < 100) {
    odometry_track_step_pos(&ot, ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset),
                            ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset));

    int s = (int)(MAX_SPEED *
                  (ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset) +
                   ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset)) /
                  200);
    s = s < MIN_SPEED ? MIN_SPEED : s;
    s = s > MAX_SPEED ? MAX_SPEED : s;
    set_speed(s, s);
    step();
  }

  // max speed
  while (ot.result.y < SCALING_TEST - 0.05) {
    odometry_track_step_pos(&ot, ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset),
                            ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset));

    set_speed(MAX_SPEED, MAX_SPEED);
    step();
  }

  // linear speed decrease over 400 increments
  while (ot.result.y < SCALING_TEST) {
    odometry_track_step_pos(&ot, ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset),
                            ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset));

    int s = (int)(MAX_SPEED * (SCALING_TEST - ot.result.y) / 0.05);
    s = s < MIN_SPEED ? MIN_SPEED : s;
    s = s > MAX_SPEED ? MAX_SPEED : s;
    set_speed(s, s);
    step();
  }

  print_position();
  printf("SCALING test finished\n");
  init();

  return;
}

int main() {
  wb_robot_init(); /* initialize the webots controller library */

  // enable keyboard
  wb_keyboard_enable(TIME_STEP);

  // get a handler to the motors and set target position to infinity (speed control).
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // get a handler to the position sensors and enable them.
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);

  // required to get the position sensor values
  wb_robot_step(TIME_STEP);

  // Initialize the modules
  odometry_track_init();
  odometry_goto_init();

  // Initializes tracking and goto structures
  odometry_track_start_pos(&ot, ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset),
                           ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset));
  odometry_goto_start(&og, &ot);

  ot.result.x = 0.0;
  ot.result.y = 0.0;
  ot.result.theta = PI / 2;

  init();

  /* main loop */
  while (true) {
    // perform a simulation step
    step();

    int key = wb_keyboard_get_key();
    int l = speed[LEFT];
    int r = speed[RIGHT];

    // move the robot with the keys
    switch (key) {
      case WB_KEYBOARD_UP:
        if (speed[LEFT] < MAX_SPEED)
          l += INCR;
        if (speed[RIGHT] < MAX_SPEED)
          r += INCR;
        break;
      case WB_KEYBOARD_DOWN:
        if (speed[LEFT] > -MAX_SPEED)
          l -= INCR;
        if (speed[RIGHT] > -(MAX_SPEED + 1))
          r -= INCR;
        break;
      case WB_KEYBOARD_LEFT:
        if (speed[RIGHT] < MAX_SPEED)
          r += INCR;
        if (speed[LEFT] > -(MAX_SPEED + 1))
          l -= INCR;
        break;
      case WB_KEYBOARD_RIGHT:
        if (speed[LEFT] < MAX_SPEED)
          l += INCR;
        if (speed[RIGHT] > -(MAX_SPEED + 1))
          r -= INCR;
        break;
      case '1':
        test_increment(INCREMENT_TEST);
        break;
      case '2':
        test_ratio((int)RATIO_TEST);
        break;
      case '3':
        test_diameter();
        break;
      case '4':
        test_scaling();
        break;
      case 'S':
        l = r = 0;
        break;
      case 'R':
        left_encoder_offset = wb_position_sensor_get_value(left_position_sensor);
        right_encoder_offset = wb_position_sensor_get_value(right_position_sensor);
        odometry_track_start_pos(&ot, ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset),
                                 ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset));
        break;
      case 'P':
        print_position();
        break;
      default:
        break;
    }

    // do the odometry step
    odometry_track_step_pos(&ot, ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset),
                            ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset));

    // send speed values to motors
    set_speed(l, r);
  }

  return 0;
}
