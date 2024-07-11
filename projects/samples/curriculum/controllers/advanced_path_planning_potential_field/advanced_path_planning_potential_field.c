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

#define PI 3.14159265358979
#define LEFT 0
#define RIGHT 1
#define VERBOSE 1
#define TIME_STEP 64
#define MAP_WIDTH 0.42
#define MAP_HEIGHT 0.30

// only one obstacle, vertices ordered clockwise
#define OBSTACLE_SIZE 3
float obstacle[OBSTACLE_SIZE][2] = {{0.07, 0.21}, {0.22, 0.09}, {0.26, 0.25}};

// wheel
#define SPEED_UNIT 0.00628
#define ENCODER_UNIT 159.23
WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;

// the goal
float goal_x = 0.36;
float goal_y = 0.245;

// attractive and repulsive factors
float k_att = 15.0;
float k_rep = 10.0;

// maximum distance of influence for the obstacles
float rho_0 = 0.037;

// speed and previous speed of the robot
int speed[2] = {0, 0};
int pspeed[2] = {0, 0};

int finished = 0;

// size of the display
int width, height;

// Instantiate odometry track structure
struct sOdometryTrack ot;

WbDeviceTag display;
WbImageRef background;

static void init_display();
static void draw_on_display();
static void set_speed(int l, int r);
static float compute_force(float pos_x, float pos_y, float *force_x, float *force_y);
static float distance_to_segment(float x, float y, float ax, float ay, float bx, float by, float *vec_x, float *vec_y);
static float distance_to_obstacle(float x, float y, float *vec_x, float *vec_y);
static int run(void);

static void init_display() {
  int i, j, color, pot;
  float fx, fy;
  width = wb_display_get_width(display);
  height = wb_display_get_height(display);

  for (i = 0; i < width; i++) {
    for (j = 0; j < height; j++) {
      pot = (int)(70 * compute_force(MAP_WIDTH * i / width, MAP_HEIGHT * j / height, &fx, &fy));
      pot = abs(pot) > 255 ? 255 : pot;
      pot = pot < 0 ? 255 : pot;
      color = 0x00010101 * (255 - pot);
      wb_display_set_color(display, color);
      wb_display_draw_rectangle(display, i, height - j - 1, 1, 1);
    }
  }

  background = wb_display_image_copy(display, 0, 0, width, height);
  wb_display_set_color(display, 0xFF0000);
}

static void draw_on_display() {
  wb_display_image_paste(display, background, 0, 0, false);

  int i = (int)(width * ot.result.x / MAP_WIDTH);
  int j = (int)(height * ot.result.y / MAP_HEIGHT);

  wb_display_draw_rectangle(display, i, height - j - 1, 1, 1);
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
 * This functions computes the force applied to the robot at point (pos_x,pos_y)
 * The result is copied in (force_x,force_y) we return the potential at the same point.
 */
static float compute_force(float pos_x, float pos_y, float *force_x, float *force_y) {
  /*
   * TODO: Complete the function such that we can compute the potenatial
   * and the applied force on the robot.
   */

  float vec_x, vec_y;
  float f_x = 0.0;
  float f_y = 0.0;
  float pot = 0.0;
  float d = distance_to_obstacle(pos_x, pos_y, &vec_x, &vec_y);
  float d_goal = sqrt((goal_x - pos_x) * (goal_x - pos_x) + (goal_y - pos_y) * (goal_y - pos_y));

  // attractive part (go to the goal)
  f_x += k_att * (goal_x - pos_x) / d_goal;
  f_y += k_att * (goal_y - pos_y) / d_goal;

  pot += k_att * d_goal * d_goal;

  // repulsive part (avoid obstacles)
  if (d < rho_0 && d != 0 && d != -1) {
    const float tx = k_rep * (1 / d - 1 / rho_0) / d * d * (-vec_x);
    const float ty = k_rep * (1 / d - 1 / rho_0) / d * d * (-vec_y);
    f_x += tx;
    f_y += ty;

    pot += k_rep * (1 / d - 1 / rho_0) * (1 / d - 1 / rho_0);
  }

  *force_x = f_x;
  *force_y = f_y;

  return pot;
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
    inObstacle = 1;

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

/*
 * This function compute the difference of two angles
 */
static float angleDiff(float angle1, float angle2) {
  // ensure both angle are withing [0, 2*PI]
  while (angle1 < 0)
    angle1 += 2 * PI;
  while (angle1 > 2 * PI)
    angle1 -= 2 * PI;
  while (angle2 < 0)
    angle2 += 2 * PI;
  while (angle2 > 2 * PI)
    angle2 -= 2 * PI;

  // compute difference
  float d = angle1 - angle2;

  while (d < -PI)
    d += 2 * PI;
  while (d > PI)
    d -= 2 * PI;

  return d;
}

/*
 * This is the main control loop function, it is called repeatedly by Webots
 */
static int run(void) {
  float fx, fy;
  float x, y, theta;

  odometry_track_step_pos(&ot, ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                          ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));
  x = ot.result.x;
  y = ot.result.y;
  theta = ot.result.theta;

  draw_on_display();

  float d_goal = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));

  if (d_goal > 0.002) {
    compute_force(x, y, &fx, &fy);
    if (VERBOSE > 0)
      printf("Current position is (%f,%f,%f), current force is (%f,%f)\n", x, y, theta, fx, fy);
    if (VERBOSE > 0)
      printf("Distance to goal is %f\n", d_goal);

    /*
     *  TODO: Write the robot control code.
     */

    float d_angle = angleDiff(atan2(fy, fx), theta);

    float ang_speed = 100 * d_angle;
    float base_speed = 15 * sqrt(fx * fx + fy * fy) * cos(d_angle);

    const int speed_l = base_speed - ang_speed;
    const int speed_r = base_speed + ang_speed;

    set_speed(speed_l, speed_r);
  } else {
    set_speed(0, 0);
    if (finished != 1) {
      printf("Goal is reached.\n");
      finished++;
    }
  }

  return TIME_STEP; /* this is the time step value, in milliseconds. */
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

  // Initializes tracking structure
  odometry_track_start_pos(&ot, ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                           ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));

  // start position
  ot.result.x = 0.04;
  ot.result.y = 0.04;
  ot.result.theta = 0;

  // get display device
  display = wb_robot_get_device("display");
  init_display();

  printf("Reset OK\n");

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1)
    run();

  wb_robot_cleanup();

  return 0;
}
