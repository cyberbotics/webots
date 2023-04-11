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
#include <stdio.h>
#include <webots/distance_sensor.h>  // distance sensor library
#include <webots/light_sensor.h>     // light sensor library
#include <webots/motor.h>
#include <webots/robot.h>  //obtain main library of webots

// Global defines
#define THRESHOLD_DIST 100
#define TIME_STEP 32        // [ms] // time step of the simulation
#define SIMULATION 0        // for robot_get_mode() function
#define CROSSCOMPILATION 1  // for robot_get_mode() function
#define REALITY 2           // for robot_get_mode() function

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
char *ps_text[] = {"one", "two", "three", "five", "seven", "nine", "ten", "eleven"};

// 8 IR light sensors
#define NB_LIGHT_SENS 8
#define LS_RIGHT_10 0
#define LS_RIGHT_45 1
#define LS_RIGHT_90 2
#define LS_RIGHT_REAR 3
#define LS_LEFT_REAR 4
#define LS_LEFT_90 5
#define LS_LEFT_45 6
#define LS_LEFT_10 7
WbDeviceTag ls[NB_LIGHT_SENS]; /* proximity sensors */
int ls_value[NB_LIGHT_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
int ls_offset_sim[NB_LIGHT_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
int ls_offset_real[NB_LIGHT_SENS] = {3239, 3122, 3581, 3763, 3847, 3814, 3747, 3538};  // to be modified according to your robot

// motors
WbDeviceTag left_motor;
WbDeviceTag right_motor;

static void reset(void) {
  int it;

  // get distance sensors
  char textPS[] = "ps0";
  for (it = 0; it < NB_DIST_SENS; it++) {
    ps[it] = wb_robot_get_device(textPS);
    textPS[2]++;
  }
  // get light sensors
  char textLS[] = "ls0";
  for (it = 0; it < NB_LIGHT_SENS; it++) {
    ls[it] = wb_robot_get_device(textLS);
    textLS[2]++;
  }

  for (it = 0; it < NB_DIST_SENS; it++)
    wb_distance_sensor_enable(ps[it], TIME_STEP);
  for (it = 0; it < NB_LIGHT_SENS; it++)
    wb_light_sensor_enable(ls[it], TIME_STEP);

  // motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

/************** Calibrate function **************
 * This function calibrates IR sensors:
 * indeed, light sensors and distance sensors
 *
 * This calibration is done over n simulation steps.
 * The results are printed in the log window
 * and are stored in the corresponded offset.
 *************************************************/
void calibrate(int n) {
  printf("\nBegin calibration.\n");

  int i, it;

  // make sure the sensors are enabled
  wb_robot_step(TIME_STEP);

  // these arrays will contain the mean of the n-1 following simulation steps
  int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  int ls_offset[NB_LIGHT_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};

  // during the following n-1 simulation steps, increment the arrays
  for (it = 0; it < n; it++) {
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_offset[i] += (int)wb_distance_sensor_get_value(ps[i]);
    for (i = 0; i < NB_LIGHT_SENS; i++)
      ls_offset[i] += (int)wb_light_sensor_get_value(ls[i]);
    wb_robot_step(TIME_STEP);
  }

  // devide each element of the array by n-1 in order to have the mean
  for (i = 0; i < NB_DIST_SENS; i++)
    ps_offset[i] /= n - 1;
  for (i = 0; i < NB_LIGHT_SENS; i++)
    ls_offset[i] /= n - 1;

  // store the answer in the right mode (either simulation or reality)
  int mode = wb_robot_get_mode();
  if (mode == SIMULATION) {
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_offset_sim[i] = ps_offset[i];
    for (i = 0; i < NB_LIGHT_SENS; i++)
      ls_offset_sim[i] = ls_offset[i];
  } else {
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_offset_real[i] = ps_offset[i];
    for (i = 0; i < NB_LIGHT_SENS; i++)
      ls_offset_real[i] = ls_offset[i];
  }

  // dislay
  printf("Distance sensor offset: %d,%d,%d,%d,%d,%d,%d,%d\n", ps_offset[0], ps_offset[1], ps_offset[2], ps_offset[3],
         ps_offset[4], ps_offset[5], ps_offset[6], ps_offset[7]);
  printf("Light sensor offset: %d,%d,%d,%d,%d,%d,%d,%d\n", ls_offset[0], ls_offset[1], ls_offset[2], ls_offset[3], ls_offset[4],
         ls_offset[5], ls_offset[6], ls_offset[7]);

  printf("Calibration is done.\n");
}

static int run(void) {
  int i;

  // 1. Obtain the correct offset according to the mode
  int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};

  int mode = wb_robot_get_mode();

  if (mode == SIMULATION) {
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_offset[i] = ps_offset_sim[i];
  } else {
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_offset[i] = ps_offset_real[i];
  }

  // 2. Get the sensors values
  for (i = 0; i < NB_DIST_SENS; i++)
    ps_value[i] = (int)wb_distance_sensor_get_value(ps[i]);
  for (i = 0; i < NB_LIGHT_SENS; i++)
    ls_value[i] = (int)wb_light_sensor_get_value(ls[7]);

  // 3. If something exceeds the threshold, print a warning in the log window
  for (i = 0; i < NB_DIST_SENS; i++) {
    if (ps_value[i] - ps_offset[i] > THRESHOLD_DIST)
      printf("An obstacle is detected at %s o'clock.\n", ps_text[i]);
  }

  /*//To uncomment for the last part of the exercise
  int left_obstacle = ps_value[6]+ps_value[7];
  int right_obstacle = ps_value[0]+ps_value[1];
  int delta = left_obstacle - right_obstacle;
  wb_motor_set_velocity(left_motor, 200+delta);
  wb_motor_set_velocity(right_motor, 200-delta);
  */

  return TIME_STEP;
}

int main() {
  wb_robot_init();

  reset();

  // calibrate(50);

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1)
    run();

  wb_robot_cleanup();

  return 0;
}
