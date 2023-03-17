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
#define MAP_SIZE 35     // N by N square grid
#define CELL_SIZE 0.03  // => X [m] width and height
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

// the odometry model (same size as the map)
float om[MAP_SIZE][MAP_SIZE];

// the sensor model (same size as the map)
float sm[MAP_SIZE][MAP_SIZE];
float sm_inv[MAP_SIZE][MAP_SIZE];  // 1 - sm

// the prediciotn
float prediction[MAP_SIZE][MAP_SIZE];

// the belief (same size as the map)
float belief[MAP_SIZE][MAP_SIZE];

// braitenberg weights for wall following
float weights_left[NB_DIST_SENS] = {-1, -1, -1, 0.5, -0.5, 0.5, 1, 2};
float weights_right[NB_DIST_SENS] = {1, 0.8, 1, -0.5, 0.5, -1, -1.6, -2};
float bias_left_follow[NB_DIST_SENS] = {0, 0, 0, 0, 0, -700, 0, 0};

// function prototypes
static void init_display();
static void draw_on_display(float max);
static int wtom(float x);
static void run();

static void init_display() {
  display = wb_robot_get_device("display");
  display_width = wb_display_get_width(display);
  display_height = wb_display_get_height(display);

  int i, j;

  wb_display_set_color(display, 0x808080);
  wb_display_fill_rectangle(display, 0, 0, display_width, display_height);
  wb_display_set_color(display, 0xFFFFFF);
  wb_display_fill_rectangle(display, 0, 0, MAP_SIZE, MAP_SIZE);

  // display the walls in black
  wb_display_set_color(display, 0x000000);
  for (i = 1; i < MAP_SIZE; i++) {
    for (j = 0; j < MAP_SIZE; j++) {
      if (map[i][j] == 1) {
        wb_display_draw_rectangle(display, i - 1, j, 1, 1);
      }
    }
  }
  background = wb_display_image_copy(display, 0, 0, display_width, display_height);
}

/*
 * This functions draw the belief on the display window
 */
static void draw_on_display(float max) {
  if (max == 0.0f)
    return;

  int i, j, g, b;

  wb_display_image_paste(display, background, 0, 0, false);
  wb_display_set_opacity(display, 0.5);

  // if the belief is uniform and 0 we plot red on all location
  for (i = 0; i < MAP_SIZE; i++) {
    for (j = 0; j < MAP_SIZE; j++) {
      if (belief[MAP_SIZE - i - 1][j] == max) {
        wb_display_set_color(display, 0x00FFFF);
        wb_display_draw_rectangle(display, i, j, 1, 1);
      } else {
        g = b = 255 * (1 - (belief[MAP_SIZE - i - 1][j] / max));
        wb_display_set_color(display, 0xFF0000 + (g << 8) + b);
        wb_display_draw_rectangle(display, i, j, 1, 1);
      }
    }
  }
  wb_display_set_opacity(display, 1.0);
}

/*
 * Those 2 functions do the coordinate transform between the world coordinates (w)
 * and the map coordinates (m) in X and Y direction.
 */
static int wtom(float x) {
  int tmp = (int)(MAP_SIZE / 2 + x / CELL_SIZE);
  if (tmp < 0)
    tmp = 0;
  if (tmp >= MAP_SIZE)
    tmp = MAP_SIZE - 1;
  return tmp;
}

/*
 * This is the run function which is called at each iteration of the main loop.
 * It will only give the motor command to the wheels.
 */
static void run(void) {
  int i;
  float speed[2] = {300, 300};

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
  int i, j, k, l;
  float max = 0.0;

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

  // Initializes tracking and goto structures
  odometry_track_start_pos(&ot, ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                           ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));

  ot.result.x = 0.0;
  ot.result.y = 0.0;
  ot.result.theta = -PI / 2;

  // map and sm init to 0  and belief to 1/MAP_SIZE^2
  for (i = 0; i < MAP_SIZE; i++) {
    for (j = 0; j < MAP_SIZE; j++) {
      map[i][j] = 0;
      sm[i][j] = 0;
      belief[i][j] = (float)1 / (float)(MAP_SIZE * MAP_SIZE);
    }
  }

  // map the arena (in map and in sm, sm_inv)
  for (i = 7; i < 29; i++) {
    map[i][2] = 1;
    sm[i][3] = 0.9;
    map[i][33] = 1;
    sm[i][32] = 0.9;
  }
  for (j = 2; j < 34; j++) {
    map[7][j] = 1;
    sm[8][j] = 0.9;
    map[28][j] = 1;
    sm[27][j] = 0.9;
  }

  // apply a kind of blur on the sensor model
  for (i = 0; i < MAP_SIZE; i++) {
    for (j = 0; j < MAP_SIZE; j++) {
      if (sm[i][j] > 0.6) {
        if (i + 1 < MAP_SIZE && sm[i + 1][j] < 0.45)
          sm[i + 1][j] = 0.45;
        if (i - 1 > 0 && sm[i - 1][j] < 0.45)
          sm[i - 1][j] = 0.45;
        if (j + 1 < MAP_SIZE && sm[i][j + 1] < 0.45)
          sm[i][j + 1] = 0.45;
        if (j - 1 > 0 && sm[i][j - 1] < 0.45)
          sm[i][j - 1] = 0.45;
      }
    }
  }
  for (i = 0; i < MAP_SIZE; i++) {
    for (j = 0; j < MAP_SIZE; j++) {
      if (sm[i][j] > 0.3 && sm[i][j] < 0.6) {
        if (i + 1 < MAP_SIZE && sm[i + 1][j] < 0.2)
          sm[i + 1][j] = 0.2;
        if (i - 1 > 0 && sm[i - 1][j] < 0.2)
          sm[i - 1][j] = 0.2;
        if (j + 1 < MAP_SIZE && sm[i][j + 1] < 0.2)
          sm[i][j + 1] = 0.2;
        if (j - 1 > 0 && sm[i][j - 1] < 0.2)
          sm[i][j - 1] = 0.2;
      }
      sm_inv[i][j] = 1 - sm[i][j];
    }
  }

  init_display();

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
  do {
    // update the odometric position
    odometry_track_step_pos(&ot, ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                            ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));

    // update position on the map
    int tmp_x = robot_x;
    robot_x = wtom(ot.result.x);
    int tmp_y = robot_y;
    robot_y = wtom(ot.result.y);

    // update belief only if cell changed
    if (robot_x != tmp_x || robot_y != tmp_y) {
      // initiate the odometry model
      for (i = 0; i < MAP_SIZE; i++) {
        for (j = 0; j < MAP_SIZE; j++) {
          om[i][j] = 0.0;
        }
      }

      // compute the odometry model (and spread it over a point
      om[robot_x][robot_y] = (float)3 / (float)19;
      if (robot_x + 1 < MAP_SIZE)
        om[robot_x + 1][robot_y] = (float)2 / (float)19;
      if (robot_x - 1 > 0)
        om[robot_x - 1][robot_y] = (float)2 / (float)19;
      if (robot_y + 1 < MAP_SIZE)
        om[robot_x][robot_y + 1] = (float)2 / (float)19;
      if (robot_y - 1 > 0)
        om[robot_x][robot_y - 1] = (float)2 / (float)19;
      if (robot_x + 2 < MAP_SIZE)
        om[robot_x + 2][robot_y] = (float)1 / (float)19;
      if (robot_x - 2 > 0)
        om[robot_x - 2][robot_y] = (float)1 / (float)19;
      if (robot_y + 2 < MAP_SIZE)
        om[robot_x][robot_y + 2] = (float)1 / (float)19;
      if (robot_y - 2 > 0)
        om[robot_x][robot_y - 2] = (float)1 / (float)19;
      if (robot_x + 1 < MAP_SIZE && robot_y + 1 < MAP_SIZE)
        om[robot_x + 1][robot_y + 1] = (float)1 / (float)19;
      if (robot_x - 1 > 0 && robot_y + 1 < MAP_SIZE)
        om[robot_x - 1][robot_y + 1] = (float)1 / (float)19;
      if (robot_x + 1 < MAP_SIZE && robot_y - 1 > 0)
        om[robot_x + 1][robot_y - 1] = (float)1 / (float)19;
      if (robot_x - 1 > 0 && robot_y - 1 > 0)
        om[robot_x - 1][robot_y - 1] = (float)1 / (float)19;

      // compute the prediction (convolution of belief and om)
      int offset_x = tmp_x;
      int offset_y = tmp_y;
      for (i = offset_x; i < MAP_SIZE + offset_x; i++) {
        for (j = offset_y; j < MAP_SIZE + offset_y; j++) {
          prediction[i - offset_x][j - offset_y] = 0.0;
          for (k = 0; k <= i; k++) {
            for (l = 0; l <= j; l++) {
              if (k > 0 && k < MAP_SIZE && l > 0 && l < MAP_SIZE && i - k > 0 && i - k < MAP_SIZE && j - l > 0 &&
                  j - l < MAP_SIZE)
                prediction[i - offset_x][j - offset_y] += belief[k][l] * om[i - k][j - l];
            }
          }
        }
      }

      max = 0.0;

      // compute the belief with prediction and sensor model
      float sum = 0.0;
      if (wb_distance_sensor_get_value(ps[5]) > THRESHOLD) {
        for (i = 0; i < MAP_SIZE; i++) {
          for (j = 0; j < MAP_SIZE; j++) {
            belief[i][j] = prediction[i][j] * sm[i][j];
            sum += belief[i][j];
            if (belief[i][j] > max)
              max = belief[i][j];
          }
        }
      } else {
        for (i = 0; i < MAP_SIZE; i++) {
          for (j = 0; j < MAP_SIZE; j++) {
            belief[i][j] = prediction[i][j] * sm_inv[i][j];
            sum += belief[i][j];
            if (belief[i][j] > max)
              max = belief[i][j];
          }
        }
      }

      // normalize the distribution
      if (sum > 0.0) {
        for (i = 0; i < MAP_SIZE; i++) {
          for (j = 0; j < MAP_SIZE; j++) {
            belief[i][j] /= sum;
          }
        }
        max /= sum;
      }
    }

    // draw belief on display
    draw_on_display(max);

    // move the robot
    run();

  } while (wb_robot_step(TIME_STEP) != -1);

  // Necessary to cleanup webots stuff
  wb_robot_cleanup();

  return 0;
}
