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
 * Description:  Implementation of the functions of the round_manager.h file
 */

#include "round_manager.h"

#include "boolean.h"
#include "helper.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/supervisor.h>

#define INPUT_FILE_NAME "input.txt"
#define WINNER_FILE_NAME "winner.txt"
#define FINAL_TIME 100
#define START_TIME 100

enum Label { FIRST_LABEL, SECOND_LABEL };

enum Mode {
  // COMMON (default):
  //  - the battery management is enabled (when the battery is empty, the robot stops)
  //  - information is displayed
  COMMON,

  // CONTEST: (COMMON mode + ...
  //  - a movie is performed
  //  - the viewpoint is move automatically
  //  - the winner is stored into the output file
  //  - webots quits at the end of the round
  CONTEST
};

// internal stuff
static enum Mode mode = COMMON;
static WbNodeRef robot_node[2] = {NULL, NULL};
static WbNodeRef vp_node = NULL;
static WbFieldRef robot_translation_field[2] = {NULL, NULL};
static WbFieldRef robot_battery[2] = {NULL, NULL};
static WbFieldRef vp_position_field = NULL;
static char *competitor[2] = {NULL, NULL};
static struct tm *date = NULL;
static double battery[] = {-1.0, -1.0};
static int counter = 0;
static int winner = -1;
static int final_counter = 0;

// read the file containing the competitors
static bool read_input_file() {
  char buffer[256];
  int i;
  FILE *file = fopen(INPUT_FILE_NAME, "r");
  if (!file)
    return false;
  int ret = true;
  for (i = 0; i < 2; i++) {
    if (fgets(buffer, 255, file) == NULL) {
      ret = false;
      break;
    }
    buffer[strlen(buffer) - 1] = 0;
    competitor[i] = malloc(strlen(buffer) + 1);  // add the final '\0'
    strcpy(competitor[i], buffer);
  }
  fclose(file);
  return ret;
}

// write the result file containing the winner id
static bool write_winner_file(bool rat0_win) {
  FILE *file = fopen(WINNER_FILE_NAME, "w");
  if (!file)
    return false;
  if (rat0_win)
    fprintf(file, "0\n");
  else
    fprintf(file, "1\n");
  fclose(file);
  return true;
}

// center the camera on the action (center of the two e-pucks)
static void move_viewpoint() {
  int i;

  const double *position[2];
  for (i = 0; i < 2; i++)
    position[i] = wb_supervisor_field_get_sf_vec3f(robot_translation_field[i]);

  double dx = position[R0][CX] - position[R1][CX];
  double dz = position[R0][CZ] - position[R1][CZ];
  double focus_x = 0.5 * (position[R0][CX] + position[R1][CX]);
  double focus_z = 0.5 * (position[R0][CZ] + position[R1][CZ]);
  double distance = sqrt(dx * dx + dz * dz);

  // hack in order to be not too close to the robots
  distance *= 1.2;

  double vp_position[3];
  vp_position[CX] = -distance + focus_x;
  vp_position[CY] = distance;
  vp_position[CZ] = -distance + focus_z;

  wb_supervisor_field_set_sf_vec3f(vp_position_field, vp_position);
}

static void init_mode() {
  mode = COMMON;
  char *ratslife_env_var = getenv("WEBOTS_RATSLIFE");
  if (ratslife_env_var) {
    if (0 == strcmp(ratslife_env_var, "CONTEST")) {
      mode = CONTEST;
      printf("Contest mode\n");
    }
  }
}

// initialize the stuff used to run the contest
void init_round() {
  int i;

  // get useful fields from the scene tree
  char node_name[] = "R0";
  for (i = 0; i < 2; i++) {
    robot_node[i] = wb_supervisor_node_get_from_def(node_name);
    robot_translation_field[i] = wb_supervisor_node_get_field(robot_node[i], "translation");
    robot_battery[i] = wb_supervisor_node_get_field(robot_node[i], "battery");
    node_name[1]++;
  }
  vp_node = wb_supervisor_node_get_from_def("VIEWPOINT");
  vp_position_field = wb_supervisor_node_get_field(vp_node, "position");

  // init the mode
  init_mode();

  // init the competitor names
  if (!read_input_file()) {
    for (i = 0; i < 2; i++) {
      competitor[i] = malloc(5);
      strcpy(competitor[i], (i == 0) ? "Rat0" : "Rat1");
    }
  }

  // get the current date
  time_t timeval;
  time(&timeval);
  date = gmtime(&timeval);
}

// main function
// called every step of the supervisor
void run_round() {
  char message[128];
  int i;

  // get the batteries
  for (i = 0; i < 2; i++)
    battery[i] = wb_supervisor_field_get_mf_float(robot_battery[i], 0);

  if (battery[0] <= 0.0 && winner != 0) {  // rat 1 wins
    snprintf(message, 128, "%s wins", competitor[R1]);
    write_winner_file(false);
    winner = 1;
    wb_supervisor_set_label(FIRST_LABEL, message, 0.0, 0.0, 0.1, WHITE, 0.0, "Arial");
    wb_supervisor_set_label(SECOND_LABEL, " ", 0.0, 0.0, 0.1, BLACK, 0.0, "Arial");
    final_counter++;
  } else if (battery[1] <= 0.0 && winner != 1) {  // rat 0 wins
    snprintf(message, 128, "%s wins", competitor[R0]);
    write_winner_file(true);
    winner = 0;
    wb_supervisor_set_label(FIRST_LABEL, message, 0.0, 0.0, 0.1, WHITE, 0.0, "Arial");
    wb_supervisor_set_label(SECOND_LABEL, " ", 0.0, 0.0, 0.1, BLACK, 0.0, "Arial");
    final_counter++;
  } else if (counter > START_TIME) {  // display batteries
    for (i = 0; i < 2; i++) {
      snprintf(message, 128, "%s: %3.2f", competitor[i], battery[i]);
      wb_supervisor_set_label(FIRST_LABEL + i, message, 0.0, 0.1 * i, 0.1, i ? GREEN : RED, 0.0, "Arial");
    }
  } else {  // startup message
    snprintf(message, 128, "%s vs %s", competitor[R0], competitor[R1]);
    wb_supervisor_set_label(FIRST_LABEL, message, 0.0, 0.0, 0.1, WHITE, 0.0, "Arial");
    snprintf(message, 128, "%d.%d.%d", date->tm_mday, date->tm_mon + 1, date->tm_year + 1900);
    wb_supervisor_set_label(SECOND_LABEL, message, 0.0, 0.1, 0.1, WHITE, 0.0, "Arial");
  }

  // CONTEST case:
  if (mode == CONTEST) {
    if (counter > START_TIME)  // follow the robots with the camera
      move_viewpoint();
    if (counter == 1)  // start the movie
      wb_supervisor_movie_start_recording("movie.avi", 640, 480, 0, 30, 1, false);
    if (final_counter > FINAL_TIME) {  // stop the movie and quit webots
      wb_supervisor_movie_stop_recording();
      wb_robot_step(0);
      wb_supervisor_simulation_quit(EXIT_SUCCESS);
    }
  }

  // increment the main counter
  counter++;
}

// clean stuff
void cleanup_round() {
  int i;
  for (i = 0; i < 2; i++)
    free(competitor[i]);
}
