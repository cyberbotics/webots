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

// Included libraries
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <webots/distance_sensor.h>  // distance sensor library
#include <webots/motor.h>            //obtain motor library
#include <webots/position_sensor.h>  //obtain position sensor library
#include <webots/robot.h>            //obtain main library of webots

// Global defines
#define THRESHOLD_DIST 300
#define TIME_STEP 32  // [ms] // time step of the simulation
#define SIMULATION 0  // for robot_get_mode() function
#define REALITY 2     // for robot_get_mode() function
#define LEFT 0        // Left side
#define RIGHT 1       // right side

// 8 IR proximity sensors
#define NB_DIST_SENS 8
#define PS_RIGHT_10 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_10 7
WbDeviceTag ps[NB_DIST_SENS]; /* proximity sensors */
int ps_value[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
int ps_offset_sim[NB_DIST_SENS] = {35, 35, 35, 35, 35, 35, 35, 35};
int ps_offset_real[NB_DIST_SENS] = {375, 158, 423, 682, 447, 594, 142, 360};  // to be modified according to your robot
int obstacle[NB_DIST_SENS];  // will contain a boolean information about obstacles

// wheel
#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.026
#define SPEED_UNIT 0.00628
WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;

// FSM
#define FORWARD 0
#define STOP 1
#define UTURN 2

// motor speeds
int speed[2] = {0, 0};
// current state
int state = FORWARD;
// timer emulation : use a counter
int n = 0;
double old_encoder = 0;

static void reset(void) {
  srand(time(0));

  int it;

  // get distance sensors
  char textPS[] = "ps0";
  for (it = 0; it < NB_DIST_SENS; it++) {
    ps[it] = wb_robot_get_device(textPS);
    textPS[2]++;
  }

  // enable distance sensor and light sensor devices
  int i;
  for (i = 0; i < NB_DIST_SENS; i++)
    wb_distance_sensor_enable(ps[i], TIME_STEP);

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
}

static int run(void) {
  // 0. Preprocessing
  // Obtain the correct offset
  int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  int i;

  if (wb_robot_get_mode() == SIMULATION) {
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_offset[i] = ps_offset_sim[i];
  } else {
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_offset[i] = ps_offset_real[i];
  }

  // 1. Get the sensors values
  // obstacle will contain a boolean information about a collision
  for (i = 0; i < NB_DIST_SENS; i++) {
    ps_value[i] = (int)wb_distance_sensor_get_value(ps[i]);
    obstacle[i] = ps_value[i] - ps_offset[i] > THRESHOLD_DIST;
  }

  // 2. Compute output values (FSM)
  switch (state) {
    case FORWARD:
      speed[LEFT] = 300;
      speed[RIGHT] = 300;
      if (obstacle[PS_RIGHT_10] || obstacle[PS_LEFT_10]) {
        state = STOP;
        n = 0;
      }
      break;
    case STOP:
      speed[LEFT] = 0;
      speed[RIGHT] = 0;
      n++;
      if (n > 40) {
        state = UTURN;
        old_encoder = fabs(wb_position_sensor_get_value(left_position_sensor));
      }
      break;
    case UTURN:
      speed[LEFT] = 150;
      speed[RIGHT] = -150;
      double new_encoder = fabs(wb_position_sensor_get_value(left_position_sensor));
      double d_step = new_encoder - old_encoder;
      double d_meter = (double)d_step * WHEEL_RADIUS;
      if (d_meter / AXLE_LENGTH > 3.14)
        state = FORWARD;
      break;
    default:
      state = FORWARD;
  }

  // 3. Send the values to actuators
  wb_motor_set_velocity(left_motor, SPEED_UNIT * speed[LEFT]);
  wb_motor_set_velocity(right_motor, SPEED_UNIT * speed[RIGHT]);

  return TIME_STEP;
}

int main() {
  wb_robot_init();

  reset();

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1)
    run();

  wb_robot_cleanup();

  return 0;
}
