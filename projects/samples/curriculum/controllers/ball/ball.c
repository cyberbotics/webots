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
#include <time.h>
#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define CONTROLLED_SPEED 0.0025
#define RANDOM_SPEED_LIMIT 0.005
#define TIME_STEP 128

float xspeed = 0.0;
float yspeed = 0.0;
int autopilot = 0;
WbNodeRef ball;
WbFieldRef ball_translation_field;
const double *ball_translation;
double new_translation[3] = {0, 0, 0};

void ensure_bounds(float *val, float min, float max) {
  if (*val < min)
    *val = min;
  else if (*val > max)
    *val = max;
}

void reset(void) {
  printf("Ball's controller started ...\n");
  printf("Move the ball with UP,DOWN,LEFT,RIGHT arrows.\n");
  printf("Random move with M key");

  /* enable keyboard feedback */
  wb_keyboard_enable(64);

  ball = wb_supervisor_node_get_from_def("BALL");
  ball_translation_field = wb_supervisor_node_get_field(ball, "translation");

  /* initialize random number generator */
  srand(time(NULL));
}

int run(void) {
  float tx = 0.0;
  float ty = 0.0;
  ball_translation = wb_supervisor_field_get_sf_vec3f(ball_translation_field);

  int key = wb_keyboard_get_key();
  while (key >= 0) {
    switch (key) {
      case WB_KEYBOARD_UP:
        tx = CONTROLLED_SPEED;
        break;
      case WB_KEYBOARD_DOWN:
        tx = -CONTROLLED_SPEED;
        break;
      case WB_KEYBOARD_LEFT:
        ty = CONTROLLED_SPEED;
        break;
      case WB_KEYBOARD_RIGHT:
        ty = -CONTROLLED_SPEED;
        break;
      case 'M':
        if (autopilot) {
          printf("Autopilot switched OFF\n");
          autopilot = 0;
        } else {
          printf("Autopilot switched ON\n");
          autopilot = 1;
        }
        break;
    }
    key = wb_keyboard_get_key();
  }

  if (autopilot) {
    /* change speed randomly */
    xspeed += (float)rand() / (float)RAND_MAX * 0.02 - 0.01;
    yspeed += (float)rand() / (float)RAND_MAX * 0.02 - 0.01;

    /* respect speed limit */
    ensure_bounds(&xspeed, -RANDOM_SPEED_LIMIT, +RANDOM_SPEED_LIMIT);
    ensure_bounds(&yspeed, -RANDOM_SPEED_LIMIT, +RANDOM_SPEED_LIMIT);

    /* add to keyboard contolled motion */
    tx += xspeed;
    ty += yspeed;
  }

  new_translation[0] = ball_translation[0] + tx;
  new_translation[1] = ball_translation[1] + ty;
  new_translation[2] = ball_translation[2];

  wb_supervisor_field_set_sf_vec3f(ball_translation_field, new_translation);

  return TIME_STEP; /* call me back after 64 milliseconds */
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
