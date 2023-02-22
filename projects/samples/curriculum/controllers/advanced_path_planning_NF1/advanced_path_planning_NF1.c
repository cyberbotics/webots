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
#include <stdio.h>
#include <stdlib.h>
#include <webots/display.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../lib/odometry.h"
#include "../../lib/odometry_goto.h"

#define PI 3.14159265358979
#define LEFT 0
#define RIGHT 1
#define EPUCK_RADIUS 0.037
#define VERBOSE 1
#define TIME_STEP 64

/*
 * define the world configuration
 * the map is a rectangle of size MAP_X * MAP_Y
 * it is modeled as RES_X * RES_Y cells
 */
#define MAP_X 0.42
#define MAP_Y 0.30
#define RES_X 42
#define RES_Y 30

// only one obstacle, vertices ordered clockwise
#define OBSTACLE_SIZE 3
float obstacle[OBSTACLE_SIZE][2] = {{0.07, 0.21}, {0.26, 0.25}, {0.22, 0.09}};

/*
 * the map structure containing occupied cells
 * the syntax is as follows :
 * -1 : obstacle
 * 0 : goal
 * >0 : distance to goal
 */
int map[RES_X][RES_Y];

// the goal (in world coordinates)
float goal_x = 0.36;
float goal_y = 0.245;

// speed and previous speed of the robot
int speed[2] = {0, 0};
int pspeed[2] = {0, 0};
float nspeed[2] = {0, 0};
int cpt = 0;

int finished = -1;

// size of the display
int width, height;

// Instantiate odometry track and goto structures
struct sOdometryTrack ot;
struct sOdometryGoto og;

// wheel
#define SPEED_UNIT 0.00628
#define ENCODER_UNIT 159.23
WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;

// display to plot the NF1 map
WbDeviceTag display;
WbImageRef background;

static void init_display();
static void draw_on_display();
static void init_map();
static int wtoc_x(float x);
static int wtoc_y(float y);
static float ctow_x(int x);
static float ctow_y(int y);
static float distance_to_segment(float x, float y, float ax, float ay, float bx, float by, float *vec_x, float *vec_y);
static float distance_to_obstacle(float x, float y, float *vec_x, float *vec_y);
static void set_cell(int x, int y, int val);
static void print_map();
static void set_speed(int l, int r);
static void goto_position(float x, float y, float theta);
static void goto_cell(int x, int y, float theta);
static void next_cell(int x, int y, int *nx, int *ny, float *ntheta);
static int run();
static void step();

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void init_display() {
  int i, j, gray;
  width = wb_display_get_width(display);
  height = wb_display_get_height(display);

  for (i = 0; i < width; i++) {
    for (j = 0; j < height; j++) {
      gray = 255 - map[i][height - j - 1];
      wb_display_set_color(display, (gray << 16) + (gray << 8) + gray);
      wb_display_draw_rectangle(display, i, j, 1, 1);
    }
  }

  background = wb_display_image_copy(display, 0, 0, width, height);
  wb_display_set_color(display, 0xFF0000);
}

static void draw_on_display() {
  wb_display_image_paste(display, background, 0, 0, false);
  wb_display_draw_rectangle(display, wtoc_x(ot.result.x), height - wtoc_y(ot.result.y) - 1, 1, 1);
}

/*
 * This functions initializes the map end computes the distances to the goal
 * for each cell using the NF1 (grassfire) algorithm.
 */
static void init_map() {
  int i, j;
  float vx, vy;

  // init to "infinity" or obstacle
  for (i = 0; i < RES_X; i++) {
    for (j = 0; j < RES_Y; j++) {
      float d = distance_to_obstacle(ctow_x(i), ctow_y(j), &vx, &vy);
      if (d < EPUCK_RADIUS && d != -1)
        map[i][j] = -1;
      else
        map[i][j] = RES_X * RES_Y;
    }
  }

  // compute goal cell
  int x = wtoc_x(goal_x);
  int y = wtoc_y(goal_y);
  printf("Goal = (%f,%f) => (%d,%d)\n", goal_x, goal_y, x, y);

  // start the NF1 recursion
  set_cell(x, y, 0);

  print_map();

  // check for out of bound values
  for (i = 0; i < RES_X; i++) {
    for (j = 0; j < RES_Y; j++) {
      if (map[i][j] < 0 || map[i][j] > 255)
        map[i][j] = 255;
    }
  }
  print_map();
  if (VERBOSE > 1)
    print_map();

  return;
}

static int wtoc_x(float x) {
  return (int)(x * RES_X / MAP_X);
}
static int wtoc_y(float y) {
  return (int)(y * RES_Y / MAP_Y);
}
static float ctow_x(int x) {
  return (x + 0.5) * MAP_X / RES_X;
}
static float ctow_y(int y) {
  return (y + 0.5) * MAP_Y / RES_Y;
}

/*
 * Compute the distance from the point (x,y) to the segment
 * determined by the points (ax,ay) and (bx,by). Returns -1 if
 * the point is not on the clockwise side of the segment. Otherwise,
 * (vec_x,vec_y) contains a unit vector representing the direction of
 * the closest point of segment.
 *
 * stores the obstacle point considered in (obs_x, obs_y) and returns the distance
 */
static float distance_to_segment(float x, float y, float ax, float ay, float bx, float by, float *vec_x, float *vec_y) {
  // lets compute useful vectors
  float v12_x = bx - ax;  // vector from 1st segment point to 2nd point
  float v12_y = by - ay;
  float v1p_x = x - ax;  // vector from 1st segment point to test point
  float v1p_y = y - ay;
  float norm_v12 = sqrt(v12_x * v12_x + v12_y * v12_y);
  float norm_v1p = sqrt(v1p_x * v1p_x + v1p_y * v1p_y);
  float dot_v12_v1p = v12_x * v1p_x + v12_y * v1p_y;

  float vx, vy, dist;
  /*
    // We can determine using the third composant of the cross product
    // if the point is on the wrong side.
    float tmp = v12_x*v1p_y - v12_y*v1p_x;
    if (tmp < 0) {
      *vec_x = 0;
      *vec_y = 0;
      return -1;
    }
  */
  // Now, we project v1p on  v12 and get the norm.
  float n = dot_v12_v1p / norm_v12;
  if (n <= 0) {
    // the closest point is the segment start
    dist = sqrt((x - ax) * (x - ax) + (y - ay) * (y - ay));  // norm([x y]-seg(1, :));
    vx = ax - x;
    vy = ay - y;  // seg(1, :) - [x y];
  } else if (n >= norm_v12) {
    // the closest point is the segment end
    dist = sqrt((x - bx) * (x - bx) + (y - by) * (y - by));  // norm([x y]-seg(2, :));
    vx = bx - x;
    vy = by - y;  // seg(2, :) - [x y];
  } else {
    // we need to project to get the closest distance
    dist = norm_v1p * sin(acos(dot_v12_v1p / norm_v12 / norm_v1p));
    vx = v12_x * dot_v12_v1p / (norm_v12 * norm_v12) - v1p_x;
    vy = v12_y * dot_v12_v1p / (norm_v12 * norm_v12) - v1p_y;  // v12 * dot_v12_v1p / (norm_v12*norm_v12) - v1p;
  }

  float norm_v = sqrt(vx * vx + vy * vy);
  if (norm_v > 0) {
    vx = vx / norm_v;
    vy = vy / norm_v;  // vec/norm(vec);
  }

  *vec_x = vx;
  *vec_y = vy;

  return dist;
}

/*
 * Compute the shortest distance from the position (x,y) to the obstacle.
 * If dist is not -1, then (vec_x, vec_y) contains the normal vector
 * pointing from (x,y) to the closest obstacle.
 */
static float distance_to_obstacle(float x, float y, float *vec_x, float *vec_y) {
  float dist = -1;
  *vec_x = 0.0;
  *vec_y = 0.0;
  float cand_x = 0.0;
  float cand_y = 0.0;
  int i = 0;
  int inObstacle = 1;

  // for each segment of the obstacle
  for (i = 0; i < OBSTACLE_SIZE; i++) {
    float d = distance_to_segment(x, y, obstacle[i][0], obstacle[i][1], obstacle[(i + 1) % OBSTACLE_SIZE][0],
                                  obstacle[(i + 1) % OBSTACLE_SIZE][1], &cand_x, &cand_y);

    if (d != -1)
      inObstacle = 0;

    if (d < dist || dist == -1) {
      dist = d;
      *vec_x = cand_x;
      *vec_y = cand_y;
    }
  }

  if (inObstacle == 1) {
    dist = -1;
    *vec_x = 0.0;
    *vec_y = 0.0;
  }

  return dist;
}

static void set_cell(int x, int y, int val) {
  map[x][y] = val;

  // recursively set all cells
  if (x + 1 < RES_X && map[x + 1][y] > val + 1)
    set_cell(x + 1, y, val + 1);
  if (y + 1 < RES_Y && map[x][y + 1] > val + 1)
    set_cell(x, y + 1, val + 1);
  if (x > 0 && map[x - 1][y] > val + 1)
    set_cell(x - 1, y, val + 1);
  if (y > 0 && map[x][y - 1] > val + 1)
    set_cell(x, y - 1, val + 1);

  return;
}

static void print_map() {
  printf("Current map :\n");
  printf("##########\n");
  int i = 0;
  for (; i < RES_X; i++) {
    int j = 0;
    for (; j < RES_Y; j++)
      printf("%d\t", map[i][j]);
    printf("\n");
  }
  printf("##########\n");

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
 * Send the robot to a relative position (x,y,theta)
 * using the "odometry" and "odometry_goto" modules
 */
static void goto_position(float x, float y, float theta) {
  // Set a target position
  odometry_goto_set_goal(&og, x, y, theta);

  // Move until the robot is close enough to the target position
  while (og.result.atgoal == 0) {
    // Update position and calculate new speeds
    odometry_track_step_pos(og.track, ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                            ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));

    if (VERBOSE > 1)
      printf("Current position = (%f,%f,%f)\n", ot.result.x, ot.result.y, ot.result.theta);

    odometry_goto_step(&og);

    // Set the wheel speed
    set_speed(og.result.speed_left, og.result.speed_right);

    draw_on_display();

    step();
  }

  return;
}

static void goto_cell(int x, int y, float theta) {
  // Set a target position
  odometry_goto_set_goal(&og, ctow_x(x), ctow_y(y), theta);

  // Move until the robot is close enough to the target position
  while (wtoc_x(ot.result.x) != x || wtoc_y(ot.result.y) != y) {
    // Update position and calculate new speeds
    odometry_track_step_pos(og.track, ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                            ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));

    if (VERBOSE > 1)
      printf("Current position = (%f,%f,%f) => (%d,%d)\n", ot.result.x, ot.result.y, ot.result.theta, wtoc_x(ot.result.x),
             wtoc_y(ot.result.y));

    odometry_goto_step(&og);

    // Set the wheel speed
    set_speed(og.result.speed_left, og.result.speed_right);

    draw_on_display();

    step();
  }

  return;
}

static void next_cell(int x, int y, int *nx, int *ny, float *ntheta) {
  int d = map[x][y];

  if (d == 0) {
    *nx = x;
    *ny = y;
    *ntheta = 0;
  } else if (x + 1 < RES_X && y + 1 < RES_Y && map[x + 1][y + 1] < d && map[x + 1][y + 1] != -1) {
    *nx = x + 1;
    *ny = y + 1;
    *ntheta = PI / 4;
  } else if (x + 1 < RES_X && y > 0 && map[x + 1][y - 1] < d && map[x + 1][y - 1] != -1) {
    *nx = x + 1;
    *ny = y - 1;
    *ntheta = -PI / 4;
  } else if (x > 0 && y + 1 < RES_Y && map[x - 1][y + 1] < d && map[x - 1][y + 1] != -1) {
    *nx = x - 1;
    *ny = y + 1;
    *ntheta = 3 * PI / 4;
  } else if (x > 0 && y > 0 && map[x - 1][y - 1] < d && map[x - 1][y - 1] != -1) {
    *nx = x - 1;
    *ny = y - 1;
    *ntheta = -3 * PI / 4;
  } else if (y + 1 < RES_Y && map[x][y + 1] < d && map[x][y + 1] != -1) {
    *nx = x;
    *ny = y + 1;
    *ntheta = PI / 2;
  } else if (x + 1 < RES_X && map[x + 1][y] < d && map[x + 1][y] != -1) {
    *nx = x + 1;
    *ny = y;
    *ntheta = 0;
  } else if (y > 0 && map[x][y - 1] < d && map[x][y - 1] != -1) {
    *nx = x;
    *ny = y - 1;
    *ntheta = -PI / 2;
  } else if (x > 0 && map[x - 1][y] < d && map[x - 1][y] != -1) {
    *nx = x - 1;
    *ny = y;
    *ntheta = -PI;
  }

  return;
}

/*
 * This is the main control loop function, it is called repeatedly by Webots
 */
static int run() {
  odometry_track_step_pos(og.track, ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                          ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));

  int x = wtoc_x(ot.result.x);
  int y = wtoc_y(ot.result.y);
  int d = map[x][y];

  int nx, ny, nnx, nny;
  float ntheta;
  next_cell(x, y, &nx, &ny, &ntheta);
  next_cell(nx, ny, &nnx, &nny, &ntheta);

  if (d == 1 && finished == -1) {
    if (VERBOSE > 0) {
      printf("Current position = (%f,%f,%f) => (%d, %d)\n", ot.result.x, ot.result.y, ot.result.theta, x, y);
      printf("Distance = %d\n", d);
      printf("Final goal : (%f,%f)\n", goal_x, goal_y);
    }
    goto_position(goal_x, goal_y, ntheta);
    finished++;
  } else if (d > 0) {
    if (VERBOSE > 0) {
      printf("Current position = (%f,%f,%f) => (%d, %d)\n", ot.result.x, ot.result.y, ot.result.theta, x, y);
      printf("Distance = %d\n", d);
      printf("Next goal : (%d,%d)\n", nx, ny);
    }
    goto_cell(nx, ny, ntheta);
  } else if (finished == 0) {
    set_speed(0, 0);
    finished++;
    printf("Final position = (%f,%f,%f)\n", ot.result.x, ot.result.y, ot.result.theta);
  }

  draw_on_display();

  return TIME_STEP;
}

/*
 * This is the main program which sets up the reset and run function.
 */
int main() {
  wb_robot_init(); /* initialize the webots controller library */

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

  // robot init speed is 0
  speed[LEFT] = speed[RIGHT] = 0;
  pspeed[LEFT] = pspeed[RIGHT] = 0;
  wb_motor_set_velocity(left_motor, SPEED_UNIT * speed[LEFT]);
  wb_motor_set_velocity(right_motor, SPEED_UNIT * speed[RIGHT]);

  // required to get the position sensor values
  wb_robot_step(TIME_STEP);

  // Initializes tracking and goto structures
  odometry_track_start_pos(&ot, ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                           ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));
  odometry_goto_start(&og, &ot);

  ot.result.x = 0.04;
  ot.result.y = 0.04;
  ot.result.theta = 0;

  init_map();

  display = wb_robot_get_device("display");
  init_display();
  draw_on_display();

  printf("Reset OK\n");

  /* main loop */
  while (true) {
    step();
    run();
  }

  return 0;
}
