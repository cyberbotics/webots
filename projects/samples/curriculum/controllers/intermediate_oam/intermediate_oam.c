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
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Global defines
#define THRESHOLD_DIST 100
#define TIME_STEP 32  // [ms] // time step of the simulation
#define SIMULATION 0  // for robot_get_mode() function
#define REALITY 2     // for robot_get_mode() function
#define LEFT 0        // Left side
#define RIGHT 1       // right side
#define NO_SIDE 2
#define SPEED_UNIT 0.00628

// motors
WbDeviceTag left_motor, right_motor;

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
int ps_offset_real[NB_DIST_SENS] = {212, 112, 149, 163, 128, 193, 111, 269};  // to be modified according to your robot
int obstacle[NB_DIST_SENS];  // will contain a boolean information about obstacles

/*****************************
 *
 *  Modules
 *
 ******************************/

// Obstacle avoidance module
#define OAM_K_90 0.08
#define OAM_K_45 0.12
#define OAM_K_10 0.12
#define OAM_K_MAX_DELTAS 700
#define WFM_MAX_SPEED 100
int oam_speed[2];
int side = NO_SIDE;
void oam(void) {
  double oam_delta = 0;
  oam_speed[LEFT] = oam_speed[RIGHT] = 0;

  if (obstacle[PS_LEFT_10] || obstacle[PS_LEFT_45] || obstacle[PS_LEFT_90]) {
    oam_delta -= (int)(OAM_K_90 * ps_value[PS_LEFT_90]);
    oam_delta -= (int)(OAM_K_45 * ps_value[PS_LEFT_45]);
    oam_delta -= (int)(OAM_K_10 * ps_value[PS_LEFT_10]);
    if (oam_delta < -WFM_MAX_SPEED)
      side = LEFT;
  } else if (obstacle[PS_RIGHT_10] || obstacle[PS_RIGHT_45] || obstacle[PS_RIGHT_90]) {
    oam_delta += (int)(OAM_K_90 * ps_value[PS_RIGHT_90]);
    oam_delta += (int)(OAM_K_45 * ps_value[PS_RIGHT_45]);
    oam_delta += (int)(OAM_K_10 * ps_value[PS_RIGHT_10]);
    if (oam_delta > WFM_MAX_SPEED)
      side = RIGHT;
  }

  if (oam_delta > OAM_K_MAX_DELTAS)
    oam_delta = OAM_K_MAX_DELTAS;
  if (oam_delta < -OAM_K_MAX_DELTAS)
    oam_delta = -OAM_K_MAX_DELTAS;

  oam_speed[LEFT] -= oam_delta;
  oam_speed[RIGHT] += oam_delta;
}

// obstacle following module
int wfm_speed[] = {0, 0};
void wfm(void) {
  switch (side) {
    case LEFT:
      wfm_speed[LEFT] = -WFM_MAX_SPEED;
      wfm_speed[RIGHT] = WFM_MAX_SPEED;
      break;
    case RIGHT:
      wfm_speed[LEFT] = WFM_MAX_SPEED;
      wfm_speed[RIGHT] = -WFM_MAX_SPEED;
      break;
    default:
      wfm_speed[LEFT] = 0;
      wfm_speed[RIGHT] = 0;
  }
}

/*****************************
 *
 *  Standard Functions
 *
 ******************************/

// Reset function
static void reset(void) {
  int it;

  // get distance sensors
  char textPS[] = "ps0";
  for (it = 0; it < NB_DIST_SENS; it++) {
    ps[it] = wb_robot_get_device(textPS);
    textPS[2]++;
  }

  // enable distance sensor devices
  int i;
  for (i = 0; i < NB_DIST_SENS; i++)
    wb_distance_sensor_enable(ps[i], TIME_STEP);

  // get a handler to the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}

static int run(void) {
  int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  int i;
  const int speed[2] = {150, 150};
  int mode = wb_robot_get_mode();

  // 1. Get the sensors values
  if (mode == SIMULATION) {
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_offset[i] = ps_offset_sim[i];
  } else {
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_offset[i] = ps_offset_real[i];
  }

  // obstacle[] will contain a boolean information about a collision
  for (i = 0; i < NB_DIST_SENS; i++) {
    ps_value[i] = (int)wb_distance_sensor_get_value(ps[i]);
    obstacle[i] = ps_value[i] - ps_offset[i] > THRESHOLD_DIST;
  }

  // 2. Behavior-based robotic:
  // call the modules in the right order
  oam();
  // wfm();

  if (ps_value[PS_RIGHT_90] - ps_offset[PS_RIGHT_90] < 0 && ps_value[PS_LEFT_90] - ps_offset[PS_LEFT_90] < 0)
    side = NO_SIDE;

  // 3. Send the values to actuators
  // send speed values to motors
  wb_motor_set_velocity(left_motor, SPEED_UNIT * (speed[LEFT] + oam_speed[LEFT] + wfm_speed[LEFT]));
  wb_motor_set_velocity(right_motor, SPEED_UNIT * (speed[RIGHT] + oam_speed[RIGHT] + wfm_speed[RIGHT]));

  return TIME_STEP;
}

int main() {
  wb_robot_init();

  reset();

  while (wb_robot_step(TIME_STEP) != -1)
    run();

  wb_robot_cleanup();

  return 0;
}
