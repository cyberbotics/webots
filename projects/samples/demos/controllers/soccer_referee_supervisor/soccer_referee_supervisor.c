/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Supevisor the soccer game from soccer.wbt
 *               Send the coordinates and orientations of each robot and the
 *               coordinates of the ball to each robot via an emitter.
 */

#include <stdio.h>
#include <string.h>
#include <webots/emitter.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define ROBOTS 6  // number of robots
#define GOAL_X_LIMIT 0.745
#define TIME_STEP 64

static void set_scores(int b, int y) {
  char score[16];

  sprintf(score, "%d", b);
  wb_supervisor_set_label(0, score, 0.92, 0.01, 0.1, 0x0000ff, 0.0, "Arial");  // blue
  sprintf(score, "%d", y);
  wb_supervisor_set_label(1, score, 0.05, 0.01, 0.1, 0xffff00, 0.0, "Arial");  // yellow
}

int main() {
  const char *robot_name[ROBOTS] = {"B1", "B2", "B3", "Y1", "Y2", "Y3"};
  WbNodeRef node;
  WbFieldRef robot_translation_field[ROBOTS], robot_rotation_field[ROBOTS], ball_translation_field;
  WbDeviceTag emitter;
  int i, j;
  int score[2] = {0, 0};
  double time = 10 * 60;  // a match lasts for 10 minutes
  double ball_reset_timer = 0;
  double ball_initial_translation[3] = {0, 0, 0};
  double robot_initial_translation[ROBOTS][3] = {{0.3, 0.03817, 0.2},  {0.3, 0.03817, -0.2},  {0.75, 0.03817, 0},
                                                 {-0.3, 0.03817, 0.2}, {-0.3, 0.03817, -0.2}, {-0.75, 0.03817, 0}};
  double robot_initial_rotation[ROBOTS][4] = {{0, 1, 0, 1.57}, {0, 1, 0, 1.57}, {0, 1, 0, 3.14},
                                              {0, 1, 0, 1.57}, {0, 1, 0, 1.57}, {0, 1, 0, 3.14}};
  double packet[ROBOTS * 3 + 2];
  char time_string[64];
  const double *robot_translation[ROBOTS], *robot_rotation[ROBOTS], *ball_translation;

  wb_robot_init();

  emitter = wb_robot_get_device("emitter");

  for (i = 0; i < ROBOTS; i++) {
    node = wb_supervisor_node_get_from_def(robot_name[i]);
    robot_translation_field[i] = wb_supervisor_node_get_field(node, "translation");
    robot_translation[i] = wb_supervisor_field_get_sf_vec3f(robot_translation_field[i]);
    for (j = 0; j < 3; j++)
      robot_initial_translation[i][j] = robot_translation[i][j];
    robot_rotation_field[i] = wb_supervisor_node_get_field(node, "rotation");
    robot_rotation[i] = wb_supervisor_field_get_sf_rotation(robot_rotation_field[i]);
    for (j = 0; j < 4; j++)
      robot_initial_rotation[i][j] = robot_rotation[i][j];
  }

  node = wb_supervisor_node_get_from_def("BALL");
  ball_translation_field = wb_supervisor_node_get_field(node, "translation");
  ball_translation = wb_supervisor_field_get_sf_vec3f(ball_translation_field);
  for (j = 0; j < 3; j++)
    ball_initial_translation[j] = ball_translation[j];
  // printf("ball initial translation = %g %g %g\n",ball_translation[0],ball_translation[1],ball_translation[2]);
  set_scores(0, 0);

  while (wb_robot_step(TIME_STEP) != -1) {
    ball_translation = wb_supervisor_field_get_sf_vec3f(ball_translation_field);
    for (i = 0; i < ROBOTS; i++) {
      robot_translation[i] = wb_supervisor_field_get_sf_vec3f(robot_translation_field[i]);
      // printf("coords for robot %d: %g %g %g\n",i,robot_translation[i][0],robot_translation[i][1],robot_translation[i][2]);
      packet[3 * i] = robot_translation[i][0];      // robot i: X
      packet[3 * i + 1] = robot_translation[i][2];  // robot i: Z

      if (robot_rotation[i][1] > 0)                // robot i: rotation Ry axis
        packet[3 * i + 2] = robot_rotation[i][3];  // robot i: alpha
      else                                         // Ry axis was inverted
        packet[3 * i + 2] = -robot_rotation[i][3];
    }
    packet[3 * ROBOTS] = ball_translation[0];      // ball X
    packet[3 * ROBOTS + 1] = ball_translation[2];  // ball Z
    wb_emitter_send(emitter, packet, sizeof(packet));

    // Adds TIME_STEP ms to the time
    time -= (double)TIME_STEP / 1000;
    if (time < 0) {
      time = 10 * 60;  // restart
    }
    sprintf(time_string, "%02d:%02d", (int)(time / 60), (int)time % 60);
    wb_supervisor_set_label(2, time_string, 0.45, 0.01, 0.1, 0x000000, 0.0, "Arial");  // black

    if (ball_reset_timer == 0) {
      if (ball_translation[0] > GOAL_X_LIMIT) {  // ball in the blue goal
        set_scores(++score[0], score[1]);
        ball_reset_timer = 3;                            // wait for 3 seconds before reseting the ball
      } else if (ball_translation[0] < -GOAL_X_LIMIT) {  // ball in the yellow goal
        set_scores(score[0], ++score[1]);
        ball_reset_timer = 3;  // wait for 3 seconds before reseting the ball
      }
    } else {
      ball_reset_timer -= (double)TIME_STEP / 1000.0;
      if (ball_reset_timer <= 0) {
        ball_reset_timer = 0;
        wb_supervisor_field_set_sf_vec3f(ball_translation_field, ball_initial_translation);
        for (i = 0; i < ROBOTS; i++) {
          wb_supervisor_field_set_sf_vec3f(robot_translation_field[i], robot_initial_translation[i]);
          wb_supervisor_field_set_sf_rotation(robot_rotation_field[i], robot_initial_rotation[i]);
        }
      }
    }
  }

  wb_robot_cleanup();

  return 0;
}
