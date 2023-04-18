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

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <webots/display.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../lib/odometry.h"

// general defines
#define LEFT 0
#define RIGHT 1
#define TIME_STEP 64
#define PI 3.141592653589793
// devices
#define NB_DIST_SENS 8
// map configuration
#define MAP_SIZE 70      // N by N square grid
#define CELL_SIZE 0.015  // => X [m] width and height
// distance threshold
#define THRESHOLD 90
// wheels
#define SPEED_UNIT 0.00628
#define ENCODER_UNIT 159.23
WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;

WbDeviceTag ps[NB_DIST_SENS];
WbDeviceTag display;
WbImageRef background;

int display_width;
int display_height;

// Instantiate odometry track and goto structures
struct sOdometryTrack ot;

// The map structure (just an integer array)
int map[MAP_SIZE][MAP_SIZE];

// the robot position on the map
int robot_x = MAP_SIZE / 2;
int robot_y = MAP_SIZE / 2;

// this is the angle at which the IR sensors are placed on the robot
float angle_offset[NB_DIST_SENS] = {0.2793, 0.7854, 1.5708, 2.618, -2.618, -1.5708, -0.7854, -0.2793};

// braitenberg weights for wall following
float weights_left[NB_DIST_SENS] = {-1, -1, -1, 0.5, -0.5, 0.5, 1, 2};
float weights_right[NB_DIST_SENS] = {1, 0.8, 1, -0.5, 0.5, -1, -1.6, -2};
float bias_left_follow[NB_DIST_SENS] = {0, 0, 0, 0, 0, -700, 0, 0};

// function prototypes
static void init_display();
static int wtom(float x);
static void occupied_cell(int x, int y, float theta);
static void run();

/*
 * Initiate the display with a white color
 */
static void init_display() {
  display = wb_robot_get_device("display");
  display_width = wb_display_get_width(display);
  display_height = wb_display_get_height(display);
  wb_display_fill_rectangle(display, 0, 0, display_width, display_height);
  background = wb_display_image_copy(display, 0, 0, display_width, display_height);
}

/*
 * Those 2 functions do the coordinate transform between the world coordinates (w)
 * and the map coordinates (m) in X and Y direction.
 */
static int wtom(float x) {
  return (int)(MAP_SIZE / 2 + x / CELL_SIZE);
}

/*
 * Set the coresponding cell to 1 (occupied)
 * and display it
 */
void occupied_cell(int x, int y, float theta) {
  // normalize angle
  while (theta > PI)
    theta -= 2 * PI;
  while (theta < -PI)
    theta += 2 * PI;

  // front cell
  if (-PI / 6 <= theta && theta <= PI / 6) {
    if (y + 1 < MAP_SIZE) {
      map[x][y + 1] = 1;
      wb_display_draw_rectangle(display, x, display_height - y, 1, 1);
    }
  }
  // right cell
  if (PI / 3 <= theta && theta <= 2 * PI / 3) {
    if (x + 1 < MAP_SIZE) {
      map[x + 1][y] = 1;
      wb_display_draw_rectangle(display, x + 1, display_height - 1 - y, 1, 1);
    }
  }
  // left cell
  if (-2 * PI / 3 <= theta && theta <= -PI / 3) {
    if (x - 1 > 0) {
      map[x - 1][y] = 1;
      wb_display_draw_rectangle(display, x - 1, display_height - 1 - y, 1, 1);
    }
  }
  // back cell
  if (5 * PI / 6 <= theta || theta <= -5 * PI / 6) {
    if (y - 1 > 0) {
      map[x][y - 1] = 1;
      wb_display_draw_rectangle(display, x, display_height - y - 2, 1, 1);
    }
  }
}

/*
 * This is the run function which is called at each iteration of the main loop.
 */
static void run(void) {
  int i;
  float speed[2] = {300, 300};

  // for each sensor if the value is above threshold we declare the
  // corresponding cell as occupied
  wb_display_image_paste(display, background, 0, 0, false);
  wb_display_set_color(display, 0x000000);
  for (i = 0; i < NB_DIST_SENS; i++) {
    if (wb_distance_sensor_get_value(ps[i]) > THRESHOLD)
      occupied_cell(robot_x, robot_y, ot.result.theta + angle_offset[i]);
  }
  wb_display_image_delete(display, background);
  background = wb_display_image_copy(display, 0, 0, display_width, display_height);

  // move avoiding obstacles
  for (i = 0; i < NB_DIST_SENS; i++) {
    float d = wb_distance_sensor_get_value(ps[i]);
    if (d > 70)
      d += bias_left_follow[i];
    if (d > 70 || d < 0) {
      speed[LEFT] += d * weights_left[i];
      speed[RIGHT] += d * weights_right[i];
    }
  }

  // limit speed in acceptable range
  speed[LEFT] = speed[LEFT] > 500 ? 500 : speed[LEFT];
  speed[LEFT] = speed[LEFT] < -500 ? -500 : speed[LEFT];
  speed[RIGHT] = speed[RIGHT] > 500 ? 500 : speed[RIGHT];
  speed[RIGHT] = speed[RIGHT] < -500 ? -500 : speed[RIGHT];

  // feed speed values to motors
  wb_motor_set_velocity(left_motor, SPEED_UNIT * speed[LEFT]);
  wb_motor_set_velocity(right_motor, SPEED_UNIT * speed[RIGHT]);
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  int i, j;

  // necessary to initialize webots stuff
  wb_robot_init();

  // initialize the random number generator
  srand(time(NULL));

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

  init_display();

  // Initializes tracking and goto structures
  odometry_track_start_pos(&ot, ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                           ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));

  ot.result.x = 0.008;
  ot.result.y = 0.008;
  ot.result.theta = 1.5731;

  // map init to 0
  for (i = 0; i < MAP_SIZE; i++) {
    for (j = 0; j < MAP_SIZE; j++)
      map[i][j] = 0;
  }

  // get the distance sensor devices
  char textPS[] = "ps0";
  for (i = 0; i < NB_DIST_SENS; i++) {
    ps[i] = wb_robot_get_device(textPS);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
    textPS[2]++;
  }

  // enables the keyboard
  wb_keyboard_enable(TIME_STEP);

  // main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // update the odometric position
    odometry_track_step_pos(&ot, ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                            ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));

    // update position on the map
    robot_x = wtom(ot.result.x);
    robot_y = wtom(ot.result.y);

    run();

    wb_display_set_color(display, 0xFF0000);
    wb_display_draw_rectangle(display, robot_x, display_height - robot_y - 1, 1, 1);
  }

  // Necessary to cleanup webots stuff
  wb_robot_cleanup();

  return 0;
}
