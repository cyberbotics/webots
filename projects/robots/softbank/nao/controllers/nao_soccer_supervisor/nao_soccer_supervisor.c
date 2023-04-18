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

//--------------------------------------------------------------------------------------------
//  Project:      Robotstadium, the online robot soccer competition
//  Description:  Supervisor controller for Robostadium/Nao soccer worlds
//                (You do not need to modify this file for the Robotstadium competition)
//                This controller has several purposes: control the current game state,
//                count the goals, display the scores, move robots and ball to kick-off
//                position, simulate the RoboCup game controller by sending data
//                to the players every 500 ms, check "kick-off shots", throw in ball
//                after it left the field, record match video, penalty kick shootout ...
//--------------------------------------------------------------------------------------------

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <webots/emitter.h>
#include <webots/keyboard.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include "RoboCupGameControlData.h"

#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif
#include <errno.h>

#define SOCKET_SERVER "127.0.0.1" /* local host */
/* port for the servers, the 10 robots must have
 * a different port number between SOCKET_PORT
 * and SOCKET_PORT + 4 (included) for the red team
 * and SOCKET_PORT + 5 and SOCKET_PORT + 9 (included)
 * for the blue team.
 */
#define SOCKET_PORT 19559

// used for indexing arrays
enum { X, Y, Z };

// max num robots
#define MAX_NUM_ROBOTS (2 * MAX_NUM_PLAYERS)

// field dimensions (in meters) according to the SPL Nao Rule Book
// clang-format off
// clang-format 11.0.0 has problems with AlignTrailingComments when the line starts with a hash (#).
#define FIELD_SIZE_X 9.050        // official size of the field
#define FIELD_SIZE_Y 6.050        // for the 2014 competition
#define CIRCLE_DIAMETER 1.550     // soccer field's central circle
#define PENALTY_SPOT_DIST 1.325   // distance between penalty spot and goal
#define PENALTY_AREA_Y_DIM 2.250  // changed according to 2014 specification
#define PENALTY_AREA_X_DIM 0.650
#define GOAL_WIDTH 1.500
#define THROW_IN_LINE_LENGTH 7.000  // total length
#define THROW_IN_LINE_OFFSET 0.400  // offset from side line
#define LINE_WIDTH 0.050            // white lines
#define BALL_RADIUS 0.0325
// clang-format on

// throw-in lines
const double THROW_IN_LINE_X_END = THROW_IN_LINE_LENGTH / 2;
const double THROW_IN_LINE_Y = FIELD_SIZE_Y / 2 - THROW_IN_LINE_OFFSET;

// ball position limits
static const double CIRCLE_RADIUS_LIMIT = CIRCLE_DIAMETER / 2 + BALL_RADIUS;
static const double FIELD_X_LIMIT = FIELD_SIZE_X / 2 + BALL_RADIUS;
static const double FIELD_Y_LIMIT = FIELD_SIZE_Y / 2 + BALL_RADIUS;

// penalties
static const double PENALTY_BALL_X_POS = FIELD_SIZE_X / 2 - PENALTY_SPOT_DIST;
static const double PENALTY_GOALIE_X_POS = (FIELD_SIZE_X - LINE_WIDTH) / 2;

// timing
static const int TIME_STEP = 40;           // should be a multiple of WorldInfo.basicTimeSTep
static const double MAX_TIME = 10 * 60.0;  // a match half lasts 10 minutes

// indices of the two robots used for penalty kick shoot-out
static const int GOALIE = 0;
static const int ATTACKER = 1;

// robot model
static const int NAO = 0;
static const int DARWIN = 1;

// waistband colors
const double RED[3] = {0.40, 0.05, 0.05};
const double BLUE[3] = {0.16, 0.70, 0.86};

// the information we need to keep about each robot
typedef struct {
  WbNodeRef robot_node;     // to track the robot node
  WbFieldRef translation;   // to track robot's position
  WbFieldRef rotation;      // to track robot's rotation
  WbFieldRef jersey_color;  // to be changed during half-time
  int model;                // NAO or DARWIN
  const double *position;   // pointer to current robot position
} Robot;

// the Robots
static Robot *robots[MAX_NUM_ROBOTS];

// zero vector
static const double ZERO_VEC_3[3] = {0, 0, 0};

// global variables
static WbNodeRef ball = NULL;                // to keep the ball node
static WbFieldRef ball_translation = NULL;   // to track ball position
static WbFieldRef ball_rotation = NULL;      // to reset ball rotation
static WbDeviceTag emitter;                  // to send game control data to robots
static WbDeviceTag receiver;                 // to receive 'move' requests
static const double *ball_pos = ZERO_VEC_3;  // current ball position (pointer to)
static double timeRemaining;                 // time [seconds] since end of half game
static int step_count = 0;                   // number of steps since the simulation started
static int last_touch_robot_index = -1;      // index of last robot that touched the ball
static const char *message;                  // instant message: "Goal!", "Out!", etc.
static double message_steps = 0;             // steps to live for instant message
static int robotInDefense[2] = {-1, -1};     // Which player is in his own penalty area for each team

// global variables used for the sudden death
static int scoredRed;
static int scoredBlue;
static int touchedRed;
static int touchedBlue;
static double timeSpentRed;
static double timeSpentBlue;
static double marginOfErrorRed;
static double marginOfErrorBlue;

// global variables for the TCP/IP communication
static struct sockaddr_in address;
static struct hostent *server;
static int attempt_frequency = 16;
static int fd[MAX_NUM_ROBOTS];
static int connection_is_established[MAX_NUM_ROBOTS];

// Robotstadium match type
enum {
  DEMO,   // DEMO, does not make a video, does not terminate the simulator, does not write scores.txt
  ROUND,  // Regular Robostadium round: 2 x 10 min + possible penalty shots
  FINAL   // Robotstadium final: add sudden death-shots if games is tied after penalty kick shoot-out
};
static int match_type = DEMO;

// default team names displayed by the Supervisor
static char team_names[2][64] = {"Red Team", "Blue Team"};

// RoboCup GameController simulation
static struct RoboCupGameControlData control_data;

// to enforce the "kick-off shot cannot score a goal" rule:
enum {
  KICK_OFF_INITIAL,  // the ball was just put in the central circle center
  KICK_OFF_PLAYED    // the ball is in play after a member of the kick-off team touches it
};
static int kick_off_state = KICK_OFF_INITIAL;

static int robot_get_teamID(int robot_index) {
  return robot_index < MAX_NUM_PLAYERS ? 0 : 1;
}

static int robot_is_blue(int robot_index) {
  return control_data.teams[robot_get_teamID(robot_index)].teamColour == TEAM_BLUE;
}

static int robot_is_red(int robot_index) {
  return control_data.teams[robot_get_teamID(robot_index)].teamColour == TEAM_RED;
}

static int robot_get_index(int robotID, int teamID) {
  if (robotID >= 0 && robotID < MAX_NUM_PLAYERS && teamID >= 0 && teamID < 2)
    return MAX_NUM_PLAYERS * teamID + robotID;
  else
    return -1;
}

static int get_blue_robot_index(int playerID) {
  return playerID + MAX_NUM_PLAYERS;
}

static int get_red_robot_index(int playerID) {
  return playerID;
}

static const char *get_team_name(int team_color) {
  return team_names[team_color];
}

// create and initialize a Robot struct
static Robot *robot_new(WbNodeRef node) {
  Robot *robot = malloc(sizeof(Robot));
  robot->robot_node = node;
  robot->translation = wb_supervisor_node_get_field(node, "translation");
  robot->rotation = wb_supervisor_node_get_field(node, "rotation");
  robot->jersey_color = wb_supervisor_node_get_field(node, "jerseyColor");
  robot->position = NULL;

  // find robot model
  WbFieldRef controllerField = wb_supervisor_node_get_field(node, "controller");
  const char *controller = wb_supervisor_field_get_sf_string(controllerField);
  if (strstr(controller, "nao"))
    robot->model = NAO;
  else
    robot->model = DARWIN;

  return robot;
}

static const char *get_robot_def_name(int robot_index) {
  static char defname[64];
  int playerID = robot_index % MAX_NUM_PLAYERS;
  int teamID = robot_get_teamID(robot_index);

  if (playerID == GOALIE)
    sprintf(defname, "GOAL_KEEPER_%d", teamID);
  else
    sprintf(defname, "PLAYER_%d_%d", playerID, teamID);

  return defname;
}

static void display() {
  const double FONT_SIZE = 0.15;

  // display team names and current score
  char text[64];
  if (control_data.firstHalf) {
    int ret = snprintf(text, sizeof(text), "%s - %d", get_team_name(TEAM_RED), control_data.teams[TEAM_RED].score);
    if (ret >= 0)
      wb_supervisor_set_label(0, text, 0.05, 0.03, FONT_SIZE, 0xec0f0f, 0.0, "Arial");  // red
    ret = snprintf(text, sizeof(text), "%d - %s", control_data.teams[TEAM_BLUE].score, get_team_name(TEAM_BLUE));
    if (ret >= 0)
      wb_supervisor_set_label(1, text, 0.99 - 0.025 * strlen(text), 0.03, FONT_SIZE, 0x0000ff, 0.0, "Arial");  // blue
  } else {
    int ret = snprintf(text, sizeof(text), "%s - %d", get_team_name(TEAM_BLUE), control_data.teams[TEAM_BLUE].score);
    if (ret >= 0)
      wb_supervisor_set_label(0, text, 0.05, 0.03, FONT_SIZE, 0x0000ff, 0.0, "Arial");  // blue
    ret = snprintf(text, sizeof(text), "%d - %s", control_data.teams[TEAM_RED].score, get_team_name(TEAM_RED));
    if (ret >= 0)
      wb_supervisor_set_label(1, text, 0.99 - 0.025 * strlen(text), 0.03, FONT_SIZE, 0xec0f0f, 0.0, "Arial");  // red
  }

  // display game state or remaining time
  if (control_data.state == STATE_PLAYING)
    snprintf(text, sizeof(text), "%02d:%02d", (int)(timeRemaining / 60), (int)timeRemaining % 60);
  else {
    static const char *STATE_NAMES[5] = {"INITIAL", "READY", "SET", "PLAYING", "FINISHED"};
    snprintf(text, sizeof(text), "%s", STATE_NAMES[control_data.state]);
  }
  wb_supervisor_set_label(2, text, 0.51 - 0.015 * strlen(text), 0.1, FONT_SIZE, 0x404040, 0.0, "Arial");  // black

  // display instant message
  if (message_steps > 0)
    wb_supervisor_set_label(3, message, 0.51 - 0.015 * strlen(message), 0.9, FONT_SIZE, 0x000000, 0.0, "Arial");  // black
  else {
    // remove instant message
    wb_supervisor_set_label(3, "", 1, 0.9, FONT_SIZE, 0x000000, 0.0, "Arial");
    message_steps = 0;
  }
}

// add an instant message
static void show_message(const char *msg) {
  message = msg;
  message_steps = 4000 / TIME_STEP;  // show message for 4 seconds
  display();
  printf("%s\n", msg);
}

// "The GameController will use TCP to connect to the robots."
static void sendGameControlData() {
  // prepare and send game control data
  control_data.secsRemaining = (uint32)timeRemaining;
  if (match_type == DEMO) {
    // ball position is not sent during official matches
    control_data.ballXPos = ball_pos[X];
    control_data.ballZPos = ball_pos[Y];
  }

  // send the data with a webots emitter for the robots using a webots receiver instead of handling TCP
  wb_emitter_send(emitter, &control_data, sizeof(control_data));

  // send the data with TCP
  int i;
  for (i = 0; i < MAX_NUM_PLAYERS; ++i) {
    if (connection_is_established[i])
      send(fd[i], (char *)&control_data, sizeof(control_data), 0);
  }
}

static void open_tcp_connections() {
#ifdef _WIN32
  TIMEVAL Timeout;
  Timeout.tv_sec = 0;
  Timeout.tv_usec = 0;
#endif

  int i;
  for (i = 0; i < MAX_NUM_ROBOTS; ++i) {
    if (!connection_is_established[i]) {
      /* fill in the socket address */
      address.sin_port = htons(SOCKET_PORT + i);

#ifdef _WIN32
      /* set socket mode in non-blocking */
      unsigned long iMode = 1;
      ioctlsocket(fd[i], FIONBIO, &iMode);
#else
      int rc =
#endif
      /* connect to the server. */
      connect(fd[i], (struct sockaddr *)&address, sizeof(struct sockaddr));

#ifdef _WIN32
      /* reset the socket to normal mode */
      iMode = 0;
      ioctlsocket(fd[i], FIONBIO, &iMode);
#else
      if (rc != -1)
        connection_is_established[i] = 1;
#endif
    }
  }

#ifdef _WIN32
  for (i = 0; i < MAX_NUM_ROBOTS; ++i) {
    if (!connection_is_established[i]) {
      fd_set Write, Err;
      FD_ZERO(&Write);
      FD_ZERO(&Err);
      FD_SET(fd[i], &Write);
      FD_SET(fd[i], &Err);

      // check if the socket is ready
      select(0, NULL, &Write, &Err, &Timeout);
      if (FD_ISSET(fd[i], &Write))
        connection_is_established[i] = 1;
    }
  }
#endif
}

// initialize devices and data
static void initialize() {
  // necessary to initialize Webots
  wb_robot_init();

  // emitter for sending game control data and receiving 'move' requests
  emitter = wb_robot_get_device("emitter");
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  // create structs for the robots present in the .wbt file
  int robot_count = 0;
  int i;
  for (i = 0; i < MAX_NUM_ROBOTS; i++) {
    WbNodeRef node = wb_supervisor_node_get_from_def(get_robot_def_name(i));
    if (node) {
      robots[i] = robot_new(node);
      robot_count++;
    } else
      robots[i] = NULL;
  }

  // to keep track of ball position
  ball = wb_supervisor_node_get_from_def("BALL");
  if (ball) {
    ball_translation = wb_supervisor_node_get_field(ball, "translation");
    ball_rotation = wb_supervisor_node_get_field(ball, "rotation");
  }

  // initialize game control data
  memset(&control_data, 0, sizeof(control_data));
  memcpy(control_data.header, GAMECONTROLLER_STRUCT_HEADER, sizeof(GAMECONTROLLER_STRUCT_HEADER) - 1);
  control_data.version = GAMECONTROLLER_STRUCT_VERSION;
  control_data.playersPerTeam = robot_count / 2;
  control_data.state = STATE_INITIAL;
  control_data.secondaryState = STATE2_NORMAL;
  control_data.teams[0].teamColour = TEAM_RED;  // does never change
  control_data.teams[1].teamColour = TEAM_BLUE;

  // eventually read teams names from file
  FILE *file = fopen("teams.txt", "r");
  if (file) {
    int n = fscanf(file, "%63[^\n]\n%63[^\n]", team_names[0], team_names[1]);
    fclose(file);
    if (n == EOF)
      fprintf(stderr, "Error while reading the \"teams.txt\" file\n");
  }

  // variable set during official matches
  const char *WEBOTS_ROBOTSTADIUM = getenv("WEBOTS_ROBOTSTADIUM");
  if (WEBOTS_ROBOTSTADIUM) {
    if (strcmp(WEBOTS_ROBOTSTADIUM, "ROUND") == 0) {
      match_type = ROUND;
      printf("Running Robotstadium ROUND match\n");
    } else if (strcmp(WEBOTS_ROBOTSTADIUM, "FINAL") == 0) {
      match_type = FINAL;
      printf("Running Robotstadium FINAL match\n");
    }
  }

  if (match_type != DEMO) {
    // start webcam script in background
    int ret = system("./webcam.php &");
    if (ret != 0)
      fprintf(stderr, "webcam.php failed\n");

    // make video: format=480x360, type=MPEG4, quality=75%
    wb_supervisor_movie_start_recording("movie.avi", 480, 360, 0, 75, 1, false);
  }

  // enable keyboard for manual score control
  wb_keyboard_enable(TIME_STEP * 10);

#ifdef _WIN32
  /* initialize the socket api */
  WSADATA info;

  int rc = WSAStartup(MAKEWORD(1, 1), &info); /* Winsock 1.1 */
  if (rc != 0) {
    printf("cannot initialize Winsock\n");

    return;
  }
#endif

  // create sockets to send TCP message to compatibles robots
  for (i = 0; i < MAX_NUM_ROBOTS; ++i) {
    /* create the socket */
    fd[i] = socket(AF_INET, SOCK_STREAM, 0);
    connection_is_established[i] = 0;

    if (fd[i] == -1) {
      printf("cannot create socket\n");

      return;
    }
  }

  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;

  server = gethostbyname((char *)SOCKET_SERVER);

  if (server)
    memcpy((char *)&address.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
  else {
    printf("cannot resolve server name: %s\n", SOCKET_SERVER);

    return;
  }

  open_tcp_connections();
}

// compute current ball velocity
static double compute_ball_velocity() {
  // ball position at previous time step
  static double x1 = 0.0;
  static double y1 = 0.0;

  // ball position at current time step
  double x2 = ball_pos[X];
  double y2 = ball_pos[Y];

  // compute ball direction
  double dx = x2 - x1;
  double dy = y2 - y1;

  // remember for the next call to this function
  x1 = x2;
  y1 = y2;

  // compute ball velocity
  return sqrt(dx * dx + dy * dy);
}

// detect if the ball has hit something (a robot, a goal post, a wall, etc.) during the last time step
// returns: 1 = hit, 0 = no hit
static int ball_has_hit_something() {
  // velocity at previous time step
  static double vel1 = 0.0;

  // current ball velocity
  double vel2 = compute_ball_velocity();

  // a strong acceleration or deceleration correspond to the ball being hit (or hitting something)
  // however some deceleration is normal because the ball slows down due to the rolling and air friction
  // filter noise: if the ball is almost still then forget it
  int hit = vel2 > 0.001 && (vel2 > vel1 * 1.2 || vel2 < vel1 * 0.8);

  // remember for next call
  vel1 = vel2;

  return hit;
}

static void check_keyboard() {
  // allow to modify score manually
  switch (wb_keyboard_get_key()) {
    case WB_KEYBOARD_SHIFT + 'R':
      control_data.teams[TEAM_RED].score--;
      display();
      break;

    case 'R':
      control_data.teams[TEAM_RED].score++;
      display();
      break;

    case WB_KEYBOARD_SHIFT + 'B':
      control_data.teams[TEAM_BLUE].score--;
      display();
      break;

    case 'B':
      control_data.teams[TEAM_BLUE].score++;
      display();
      break;
  }
}

// euler-axes-angle (vrml) to quaternion conversion
static void vrml_to_q(const double v[4], double q[4]) {
  double l = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  if (l > 0.0) {
    q[0] = cos(v[3] / 2);
    l = sin(v[3] / 2) / sqrt(l);
    q[1] = v[0] * l;
    q[2] = v[1] * l;
    q[3] = v[2] * l;
  } else {
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
  }
}

// quaternion to euler-axes-angle (vrml) conversion
static void q_to_vrml(const double q[4], double v[4]) {
  // if q[0] > 1, acos will return nan
  // if this actually happens we should normalize the quaternion here
  v[3] = 2.0 * acos(q[0]);
  if (v[3] < 0.0001) {
    // if e[3] close to zero then direction of axis not important
    v[0] = 0.0;
    v[1] = 1.0;
    v[2] = 0.0;
  } else {
    // normalise axes
    double n = sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    v[0] = q[1] / n;
    v[1] = q[2] / n;
    v[2] = q[3] / n;
  }
}

// quaternion multiplication (combining rotations)
static void q_mult(double qa[4], const double qb[4], const double qc[4]) {
  qa[0] = qb[0] * qc[0] - qb[1] * qc[1] - qb[2] * qc[2] - qb[3] * qc[3];
  qa[1] = qb[0] * qc[1] + qb[1] * qc[0] + qb[2] * qc[3] - qb[3] * qc[2];
  qa[2] = qb[0] * qc[2] + qb[2] * qc[0] + qb[3] * qc[1] - qb[1] * qc[3];
  qa[3] = qb[0] * qc[3] + qb[3] * qc[0] + qb[1] * qc[2] - qb[2] * qc[1];
}

// move robot to a 3d position
static void move_robot_3d(int robot_index, double tx, double ty, double tz, double alpha) {
  if (robots[robot_index]) {
    // set translation
    double trans[3] = {tx, ty, tz};
    wb_supervisor_field_set_sf_vec3f(robots[robot_index]->translation, trans);

    // set rotation
    if (robots[robot_index]->model == NAO) {
      // in NAO case we need to add a rotation before
      double rot1[4] = {1, 0, 0, 0};
      double rot2[4] = {0, 0, 1, alpha};

      // convert to quaternions
      double q1[4], q2[4], qr[4], rr[4];
      vrml_to_q(rot1, q1);
      vrml_to_q(rot2, q2);

      // multiply quaternions
      q_mult(qr, q2, q1);

      // convert to VRML
      q_to_vrml(qr, rr);

      wb_supervisor_field_set_sf_rotation(robots[robot_index]->rotation, rr);
    } else {
      // in robotis-op2 case we need to add a rotation before
      double rot1[4] = {0, 1, 0, 0.12};
      double rot2[4] = {0, 0, 1, alpha};

      // convert to quaternions
      double q1[4], q2[4], qr[4], rr[4];
      vrml_to_q(rot1, q1);
      vrml_to_q(rot2, q2);

      // multiply quaternions
      q_mult(qr, q2, q1);

      // convert to VRML
      q_to_vrml(qr, rr);

      wb_supervisor_field_set_sf_rotation(robots[robot_index]->rotation, rr);
    }
    wb_supervisor_node_reset_physics(robots[robot_index]->robot_node);
  }
}

// place robot in upright position, feet on the floor, facing
static void move_robot_2d(int robot_index, double tx, double ty, double alpha) {
  if (robots[robot_index] && robots[robot_index]->model == NAO)
    move_robot_3d(robot_index, tx, ty, 0.35, alpha);
  if (robots[robot_index] && robots[robot_index]->model == DARWIN)
    move_robot_3d(robot_index, tx, ty, 0.25, alpha);
}

// move ball to 3d position
static void move_ball_3d(double tx, double ty, double tz) {
  if (ball_translation && ball_rotation) {
    double trans[3] = {tx, ty, tz};
    double rot[4] = {0, 1, 0, 0};
    wb_supervisor_field_set_sf_vec3f(ball_translation, trans);
    wb_supervisor_field_set_sf_rotation(ball_rotation, rot);
    wb_supervisor_node_reset_physics(ball);
  }
}

// move ball to 2d position and down on the floor
static void move_ball_2d(double tx, double ty) {
  move_ball_3d(tx, ty, BALL_RADIUS);
}

// remove penalized robot from the field
static void penalize_robot(int robot_index) {
  double xTranslation = ((0 < robot_index - MAX_NUM_PLAYERS) - (robot_index - MAX_NUM_PLAYERS < 0)) * 10.0;
  double yTranslation = -5 + 2 * (robot_index - robot_get_teamID(robot_index) * MAX_NUM_PLAYERS);
  double zTranslation = wb_supervisor_field_get_sf_vec3f(robots[robot_index]->translation)[Y];

  double out_of_field[3] = {xTranslation, yTranslation, zTranslation};
  wb_supervisor_field_set_sf_vec3f(robots[robot_index]->translation, out_of_field);
}

// check if a robot is near the 2d position (x, y)
static int is_robot_near(double x, double y) {
  int i;
  for (i = 0; i < MAX_NUM_ROBOTS; ++i) {
    if (robots[i]) {
      double dx = robots[i]->position[X] - x;
      double dy = robots[i]->position[Y] - y;

      // squared distance between robot and ball
      double dist2 = sqrt(dx * dx + dy * dy);
      if (dist2 < 0.4)  // we want 40 cm free around
        return 1;
    }
  }

  return 0;
}

// put robot on the field at the end of the penalty
static void remove_penalty(int robot_index) {
  int zone1 = 1;
  int zone2 = 0;
  int zone3 = 0;
  int zone4 = 0;
  double xEntryPoint = ((0 < robot_index - MAX_NUM_PLAYERS) - (robot_index - MAX_NUM_PLAYERS < 0)) * PENALTY_BALL_X_POS;
  if (!control_data.firstHalf)
    xEntryPoint *= -1;
  double yEntryPoint = -FIELD_Y_LIMIT;

  // try to find a possible position if the default one is occupied
  while (is_robot_near(xEntryPoint, yEntryPoint)) {
    if (zone1) {
      xEntryPoint += 0.2 * ((0 < robot_index - MAX_NUM_PLAYERS) - (robot_index - MAX_NUM_PLAYERS < 0));
      if (fabs(xEntryPoint) > FIELD_X_LIMIT) {
        xEntryPoint = ((0 < robot_index - MAX_NUM_PLAYERS) - (robot_index - MAX_NUM_PLAYERS < 0)) * PENALTY_BALL_X_POS;
        if (!control_data.firstHalf)
          xEntryPoint *= -1;
        yEntryPoint = FIELD_Y_LIMIT;
        zone1 = 0;
        zone2 = 1;
      }
    } else if (zone2) {
      xEntryPoint += 0.2 * ((0 < robot_index - MAX_NUM_PLAYERS) - (robot_index - MAX_NUM_PLAYERS < 0));
      if (fabs(xEntryPoint) > FIELD_X_LIMIT) {
        xEntryPoint = ((0 < robot_index - MAX_NUM_PLAYERS) - (robot_index - MAX_NUM_PLAYERS < 0)) * PENALTY_BALL_X_POS;
        if (!control_data.firstHalf)
          xEntryPoint *= -1;
        yEntryPoint = -FIELD_Y_LIMIT;
        zone2 = 0;
        zone3 = 1;
      }
    } else if (zone3) {
      xEntryPoint -= 0.2 * ((0 < robot_index - MAX_NUM_PLAYERS) - (robot_index - MAX_NUM_PLAYERS < 0));
      if (fabs(xEntryPoint) <= 0.2) {
        xEntryPoint = ((0 < robot_index - MAX_NUM_PLAYERS) - (robot_index - MAX_NUM_PLAYERS < 0)) * PENALTY_BALL_X_POS;
        if (!control_data.firstHalf)
          xEntryPoint *= -1;
        yEntryPoint = FIELD_Y_LIMIT;
        zone3 = 0;
        zone4 = 1;
      }
    } else if (zone4) {
      xEntryPoint -= 0.2 * ((0 < robot_index - MAX_NUM_PLAYERS) - (robot_index - MAX_NUM_PLAYERS < 0));
      if (fabs(xEntryPoint) <= 0.2) {
        xEntryPoint = ((0 < robot_index - MAX_NUM_PLAYERS) - (robot_index - MAX_NUM_PLAYERS < 0)) * PENALTY_BALL_X_POS;
        if (!control_data.firstHalf)
          xEntryPoint *= -1;
        yEntryPoint = -FIELD_Y_LIMIT;
        zone4 = 0;
        zone1 = 1;
      }
    }
  }

  if (yEntryPoint < 0)
    move_robot_2d(robot_index, xEntryPoint, yEntryPoint, -M_PI_2);
  else
    move_robot_2d(robot_index, xEntryPoint, yEntryPoint, M_PI_2);
}

// handles a "move robot" request received from a robot controller
static void handle_move_robot_request(const char *request) {
  if (match_type != DEMO) {
    fprintf(stderr, "not in DEMO mode: ignoring request: %s\n", request);
    return;
  }

  int robotID, teamID;
  double tx, ty, tz, alpha;
  if (sscanf(request, "move robot %d %d %lf %lf %lf %lf", &robotID, &teamID, &tx, &ty, &tz, &alpha) != 6) {
    fprintf(stderr, "unexpected number of arguments in 'move robot' request: %s\n", request);
    return;
  }

  // move it now!
  printf("executing: %s\n", request);
  int robot_index = robot_get_index(robotID, teamID);
  if (robot_index != -1 && robots[robot_index])
    move_robot_3d(robot_index, tx, ty, tz, alpha);
  else
    fprintf(stderr, "no such robot: %d %d\n", robotID, teamID);
}

// handle a "move ball" request received from a robot controller
static void handle_move_ball_request(const char *request) {
  if (match_type != DEMO) {
    fprintf(stderr, "not in DEMO mode: ignoring request: %s\n", request);
    return;
  }

  double tx, ty, tz;
  if (sscanf(request, "move ball %lf %lf %lf", &tx, &ty, &tz) != 3) {
    fprintf(stderr, "unexpected number of arguments in 'move ball' request: %s\n", request);
    return;
  }

  // move it now!
  printf("executing: %s\n", request);
  move_ball_3d(tx, ty, tz);
}

static void read_incoming_messages() {
  // read while queue not empty
  while (wb_receiver_get_queue_length(receiver) > 0) {
    // I'm only expecting ascii messages
    const char *request = wb_receiver_get_data(receiver);
    if (memcmp(request, "move robot ", 11) == 0)
      handle_move_robot_request(request);
    else if (memcmp(request, "move ball ", 10) == 0)
      handle_move_ball_request(request);
    else
      fprintf(stderr, "received unknown message of %d bytes\n", wb_receiver_get_data_size(receiver));

    wb_receiver_next_packet(receiver);
  }
}

// this is what is done at every time step independently of the game state
static void step() {
  // copy pointer to ball position values
  if (ball_translation)
    ball_pos = wb_supervisor_field_get_sf_vec3f(ball_translation);

  // update robot position pointers
  int i;
  for (i = 0; i < MAX_NUM_ROBOTS; i++)
    if (robots[i])
      robots[i]->position = wb_supervisor_field_get_sf_vec3f(robots[i]->translation);

  if (message_steps)
    message_steps--;

  // yield control to simulator
  wb_robot_step(TIME_STEP);

  // every 480 milliseconds
  if (step_count % 12 == 0)
    sendGameControlData();

  // try periodically to open tcp connections with robots where it's not the case
  if (step_count % attempt_frequency == 0) {
    open_tcp_connections();

    // cppcheck-suppress knownConditionTrueFalse
    if (attempt_frequency < 256)
      attempt_frequency *= 2;
  }

  step_count++;

  // did I receive a message ?
  read_incoming_messages();

  // read key pressed
  check_keyboard();
}

// move robots and ball to kick-off position
static void place_to_kickoff() {
  // Manual placement according to the RoboCup SPL Rule Book:
  //   "The kicking-off robot is placed on the center circle, right in front of the penalty mark.
  //   Its feet touch the line, but they are not inside the center circle.
  //   The second field player of the attacking team is placed in front
  //   of one of the goal posts on the height of the penalty mark"

  double KICK_OFF_X = CIRCLE_DIAMETER / 2 + LINE_WIDTH;
  double KICK_OFF_Y = (PENALTY_AREA_Y_DIM + FIELD_SIZE_Y) / 4.0;
  double PENALTY_X_POS = PENALTY_BALL_X_POS;
  double GOALIE_X = (FIELD_SIZE_X - LINE_WIDTH) / 2.0;
  double DEFENDER_X = FIELD_SIZE_X / 2 - PENALTY_AREA_X_DIM - LINE_WIDTH;
  double DEFENDER_OFFSET_X = 0.5;
  double M_PI_BLUE = M_PI;
  double M_PI_RED = 0;

  if (!control_data.firstHalf) {
    KICK_OFF_X *= -1;
    GOALIE_X *= -1;
    DEFENDER_X *= -1;
    DEFENDER_OFFSET_X *= -1;
    PENALTY_X_POS *= -1;
    M_PI_BLUE = 0;
    M_PI_RED = M_PI;
  }

  // move the two goalies
  move_robot_2d(get_red_robot_index(0), -GOALIE_X, 0, M_PI_RED);
  move_robot_2d(get_blue_robot_index(0), GOALIE_X, 0, M_PI_BLUE);

  // move other robots
  if (control_data.kickOffTeam == TEAM_BLUE) {
    move_robot_2d(get_blue_robot_index(1), KICK_OFF_X, 0, M_PI_BLUE);
    move_robot_2d(get_blue_robot_index(2), PENALTY_X_POS, GOAL_WIDTH / 2, M_PI_BLUE);
    move_robot_2d(get_blue_robot_index(3), DEFENDER_X - DEFENDER_OFFSET_X / 2.0, -PENALTY_AREA_Y_DIM / 2, M_PI_BLUE);
    move_robot_2d(get_blue_robot_index(4), DEFENDER_X - DEFENDER_OFFSET_X / 2.0, PENALTY_AREA_Y_DIM / 2, M_PI_BLUE);

    // The 0.5 diff is to avoid provoking the illegal defender rule immediately
    move_robot_2d(get_red_robot_index(1), -DEFENDER_X + DEFENDER_OFFSET_X, GOAL_WIDTH / 4.0, -M_PI_RED);
    move_robot_2d(get_red_robot_index(2), -DEFENDER_X + DEFENDER_OFFSET_X, -GOAL_WIDTH / 4.0, -M_PI_RED);
    move_robot_2d(get_red_robot_index(3), -DEFENDER_X, KICK_OFF_Y, M_PI_RED);
    move_robot_2d(get_red_robot_index(4), -DEFENDER_X, -KICK_OFF_Y, M_PI_RED);
  } else {
    move_robot_2d(get_blue_robot_index(1), DEFENDER_X - DEFENDER_OFFSET_X, GOAL_WIDTH / 4.0, -M_PI_BLUE);
    move_robot_2d(get_blue_robot_index(2), DEFENDER_X - DEFENDER_OFFSET_X, -GOAL_WIDTH / 4.0, -M_PI_BLUE);
    move_robot_2d(get_blue_robot_index(3), DEFENDER_X, KICK_OFF_Y, M_PI_BLUE);
    move_robot_2d(get_blue_robot_index(4), DEFENDER_X, -KICK_OFF_Y, M_PI_BLUE);

    move_robot_2d(get_red_robot_index(1), -KICK_OFF_X, 0, M_PI_RED);
    move_robot_2d(get_red_robot_index(2), -PENALTY_X_POS, -GOAL_WIDTH / 2, M_PI_RED);
    move_robot_2d(get_red_robot_index(3), -DEFENDER_X + DEFENDER_OFFSET_X / 2.0, -PENALTY_AREA_Y_DIM / 2, M_PI_RED);
    move_robot_2d(get_red_robot_index(4), -DEFENDER_X + DEFENDER_OFFSET_X / 2.0, PENALTY_AREA_Y_DIM / 2, M_PI_RED);
  }

  // reset ball position
  move_ball_2d(0, 0);
}

// run simulation for the specified number of seconds
static void run_seconds(double seconds) {
  int n = 1000.0 * seconds / TIME_STEP;
  int i;
  for (i = 0; i < n; i++)
    step();
}

static void hold_to_kickoff(double seconds) {
  int n = 1000.0 * seconds / TIME_STEP;
  int i;
  for (i = 0; i < n; i++) {
    place_to_kickoff();
    step();
  }
}

static void run_initial_state() {
  timeRemaining = MAX_TIME;
  control_data.state = STATE_INITIAL;
  display();
  hold_to_kickoff(1.5);
}

static void run_ready_state() {
  control_data.state = STATE_READY;
  display();
  run_seconds(1.5);
}

static void run_set_state() {
  control_data.state = STATE_SET;
  display();
  hold_to_kickoff(1.5);
}

static void run_finished_state() {
  control_data.state = STATE_FINISHED;
  display();
  run_seconds(1.5);
}

static int is_in_kickoff_team(int robot_index) {
  if (control_data.kickOffTeam == TEAM_RED && robot_is_red(robot_index))
    return 1;
  if (control_data.kickOffTeam == TEAM_BLUE && robot_is_blue(robot_index))
    return 1;
  return 0;
}

// detect if a robot has just touched the ball (in the last time step)
// returns: robot index or -1 if there is no such robot
static int detect_ball_touch() {
  if (!ball_has_hit_something())
    return -1;

  // find which robot is the closest to the ball
  double minDist2 = 0.25;  // squared robot proximity radius
  int index = -1;
  int i;
  for (i = 0; i < MAX_NUM_ROBOTS; i++) {
    if (robots[i]) {
      double dx = robots[i]->position[X] - ball_pos[X];
      double dy = robots[i]->position[Y] - ball_pos[Y];

      // squared distance between robot and ball
      double dist2 = dx * dx + dy * dy;
      if (dist2 < minDist2) {
        minDist2 = dist2;
        index = i;
      }
    }
  }

  // print info
  if (index > -1) {
    if (robot_is_red(index))
      printf("RED TOUCH\n");
    else
      printf("BLUE TOUCH\n");
  }

  return index;
}

static int is_ball_in_field() {
  return fabs(ball_pos[Y]) <= FIELD_Y_LIMIT && fabs(ball_pos[X]) <= FIELD_X_LIMIT;
}

static int is_ball_in_red_goal() {
  if (control_data.firstHalf)
    return ball_pos[X] < -FIELD_X_LIMIT && ball_pos[X] > -(FIELD_X_LIMIT + 0.25) && fabs(ball_pos[Y]) < GOAL_WIDTH / 2;
  else
    return ball_pos[X] > FIELD_X_LIMIT && ball_pos[X] < FIELD_X_LIMIT + 0.25 && fabs(ball_pos[Y]) < GOAL_WIDTH / 2;
}

static int is_ball_in_blue_goal() {
  if (control_data.firstHalf)
    return ball_pos[X] > FIELD_X_LIMIT && ball_pos[X] < FIELD_X_LIMIT + 0.25 && fabs(ball_pos[Y]) < GOAL_WIDTH / 2;
  else
    return ball_pos[X] < -FIELD_X_LIMIT && ball_pos[X] > -(FIELD_X_LIMIT + 0.25) && fabs(ball_pos[Y]) < GOAL_WIDTH / 2;
}

static int is_robot_in_central_circle(int robot_index) {
  return (robots[robot_index]->position[X] * robots[robot_index]->position[X] +
          robots[robot_index]->position[Y] * robots[robot_index]->position[Y]) < CIRCLE_RADIUS_LIMIT * CIRCLE_RADIUS_LIMIT;
}

static int is_robot_in_own_penalty_area(int robot_index) {
  if (((robot_get_teamID(robot_index) == TEAM_BLUE) && control_data.firstHalf) ||
      ((robot_get_teamID(robot_index) == TEAM_RED) && !control_data.firstHalf)) {
    return robots[robot_index]->position[X] < FIELD_X_LIMIT &&
           robots[robot_index]->position[X] > FIELD_X_LIMIT - PENALTY_AREA_X_DIM &&
           fabs(robots[robot_index]->position[Y]) < PENALTY_AREA_Y_DIM / 2;
  } else {
    return robots[robot_index]->position[X] > -FIELD_X_LIMIT &&
           robots[robot_index]->position[X] < -(FIELD_X_LIMIT - PENALTY_AREA_X_DIM) &&
           fabs(robots[robot_index]->position[Y]) < PENALTY_AREA_Y_DIM / 2;
  }
}

static double sign(double value) {
  return value > 0.0 ? 1.0 : -1.0;
}

static void update_kick_off_state(double startTime, double currentTime) {
  // "The ball is in play once it is touched by the attacking team or
  // once 10 seconds have elapsed in the playing state."
  int touch_index = detect_ball_touch();
  if (touch_index != -1) {
    last_touch_robot_index = touch_index;

    if (is_in_kickoff_team(last_touch_robot_index))
      kick_off_state = KICK_OFF_PLAYED;
  } else if (startTime - currentTime > 10) {
    kick_off_state = KICK_OFF_PLAYED;
  }
}

// check if throwing the ball in does not collide with a robot.
// If it does collide, change the throw-in location.
static void check_throw_in(double x, double y) {
  // run some steps to see if the ball is moving: that would indicate a collision
  step();
  ball_has_hit_something();
  step();
  ball_has_hit_something();  // because after a throw in, this method return always 1 even if no collision occured
  step();

  while (ball_has_hit_something()) {
    // slope of the line formed by the throw in point and the origin point.
    double slope = y / x;
    y -= sign(y) * 0.1;
    x = y / slope;
    move_ball_2d(x, y);
    check_throw_in(x, y);  // recursive call to check the new throw in point.
  }
}

// check if the ball leaves the field and throw ball in if necessary
static void check_ball_out() {
  double throw_in_pos[3];  // x and z throw-in position

  if (fabs(ball_pos[Y]) > FIELD_Y_LIMIT) {  // out at side line
    // printf("ball over side-line: %f %f\n", ball_pos[X], ball_pos[Y]);
    double back;
    if (last_touch_robot_index == -1)  // not sure which team has last touched the ball
      back = 0.0;
    else if (robot_is_red(last_touch_robot_index))
      back = 1.0;  // 1 meter towards red goal
    else
      back = -1.0;  // 1 meter towards blue goal

    throw_in_pos[X] = ball_pos[X] + back;
    throw_in_pos[Y] = sign(ball_pos[Y]) * THROW_IN_LINE_Y;

    // in any case the ball cannot be placed off the throw-in line
    if (throw_in_pos[X] > THROW_IN_LINE_X_END)
      throw_in_pos[X] = THROW_IN_LINE_X_END;
    else if (throw_in_pos[X] < -THROW_IN_LINE_X_END)
      throw_in_pos[X] = -THROW_IN_LINE_X_END;
  } else if (ball_pos[X] > FIELD_X_LIMIT && !is_ball_in_red_goal() && !is_ball_in_blue_goal()) {  // out at end line
    // printf("ball over end-line (near red goal): %f %f\n", ball_pos[X], ball_pos[Y]);
    if (last_touch_robot_index == -1) {  // not sure which team has last touched the ball
      throw_in_pos[X] = THROW_IN_LINE_X_END;
      throw_in_pos[Y] = sign(ball_pos[Y]) * THROW_IN_LINE_Y;
    } else if (robot_is_red(last_touch_robot_index)) {  // defensive team
      throw_in_pos[X] = THROW_IN_LINE_X_END;
      throw_in_pos[Y] = sign(ball_pos[Y]) * THROW_IN_LINE_Y;
    } else {                  // offensive team
      throw_in_pos[X] = 0.0;  // halfway line
      throw_in_pos[Y] = sign(ball_pos[Y]) * THROW_IN_LINE_Y;
    }
  } else if (ball_pos[X] < -FIELD_X_LIMIT && !is_ball_in_blue_goal() && !is_ball_in_red_goal()) {  // out at end line
    // printf("ball over end-line (near blue goal): %f %f\n", ball_pos[X], ball_pos[Y]);
    if (last_touch_robot_index == -1) {  // not sure which team has last touched the ball
      throw_in_pos[X] = -THROW_IN_LINE_X_END;
      throw_in_pos[Y] = sign(ball_pos[Y]) * THROW_IN_LINE_Y;
    } else if (robot_is_blue(last_touch_robot_index)) {  // defensive team
      throw_in_pos[X] = -THROW_IN_LINE_X_END;
      throw_in_pos[Y] = sign(ball_pos[Y]) * THROW_IN_LINE_Y;
    } else {                  // offensive team
      throw_in_pos[X] = 0.0;  // halfway line
      throw_in_pos[Y] = sign(ball_pos[Y]) * THROW_IN_LINE_Y;
    }
  } else
    return;  // ball is not out

  // the ball is out:
  show_message("OUT!");
  kick_off_state = KICK_OFF_PLAYED;

  // let the ball roll for 2 seconds
  run_seconds(2.0);

  // throw the ball in
  move_ball_2d(throw_in_pos[X], throw_in_pos[Y]);
  check_throw_in(throw_in_pos[X], throw_in_pos[Y]);
}

// "Only the goal keeper and at most one defending field player can
// be within a team’s penalty area." (Penalty: Illegal Defender)
static void check_position_defenders() {
  int i;
  int alreadyRedInZone = robotInDefense[TEAM_RED] != -1;
  int alreadyBlueInZone = robotInDefense[TEAM_BLUE] != -1;

  // check if the red robot in the red penalty zone is still there
  if (alreadyRedInZone) {
    if (!is_robot_in_own_penalty_area(get_red_robot_index(robotInDefense[TEAM_RED])))
      robotInDefense[TEAM_RED] = -1;
  }
  // check if the blue robot in the blue penalty zone is still there
  if (alreadyBlueInZone) {
    if (!is_robot_in_own_penalty_area(get_blue_robot_index(robotInDefense[TEAM_BLUE])))
      robotInDefense[TEAM_BLUE] = -1;
  }

  for (i = 1; i < MAX_NUM_PLAYERS; ++i) {  // the goalkeeper doesn't count towards the limit
    // check for the red robots
    if (robots[get_red_robot_index(i)] && i != robotInDefense[TEAM_RED] &&
        is_robot_in_own_penalty_area(get_red_robot_index(i))) {
      if (!alreadyRedInZone) {
        robotInDefense[TEAM_RED] = i;
        alreadyRedInZone = 1;
      } else {
        char msg[] = "Illegal Defender (Red X)";
        msg[22] = (char)(((int)'0') + i + 1);
        show_message(msg);
        control_data.teams[TEAM_RED].players[i].penalty = PENALTY_ILLEGAL_DEFENDER;
        control_data.teams[TEAM_RED].players[i].secsTillUnpenalised = PENALTY_TIME;
        penalize_robot(get_red_robot_index(i));
      }
    }

    // check for the blue robots
    if (robots[get_blue_robot_index(i)] && i != robotInDefense[TEAM_BLUE] &&
        is_robot_in_own_penalty_area(get_blue_robot_index(i))) {
      if (!alreadyBlueInZone) {
        robotInDefense[TEAM_BLUE] = i;
        alreadyBlueInZone = 1;
      } else {
        char msg[] = "Illegal Defender (Blue X)";
        msg[23] = (char)(((int)'0') + i + 1);
        show_message(msg);
        control_data.teams[TEAM_BLUE].players[i].penalty = PENALTY_ILLEGAL_DEFENDER;
        control_data.teams[TEAM_BLUE].players[i].secsTillUnpenalised = PENALTY_TIME;
        penalize_robot(get_blue_robot_index(i));
      }
    }
  }
}

static void run_playing_state() {
  control_data.state = STATE_PLAYING;
  show_message("KICK-OFF!");
  double startingTime = timeRemaining;
  kick_off_state = KICK_OFF_INITIAL;
  last_touch_robot_index = -1;

  while (1) {
    // substract TIME_STEP to current time
    timeRemaining -= TIME_STEP / 1000.0;
    display();

    if (timeRemaining < 0.0) {
      timeRemaining = 0.0;
      control_data.state = STATE_FINISHED;
      return;
    }

    int i;
    for (i = 0; i < MAX_NUM_PLAYERS; ++i) {
      if (robots[get_red_robot_index(i)] && control_data.teams[TEAM_RED].players[i].secsTillUnpenalised > 0) {
        control_data.teams[TEAM_RED].players[i].secsTillUnpenalised -= TIME_STEP / 1000.0;
        if (control_data.teams[TEAM_RED].players[i].secsTillUnpenalised <= 0) {
          control_data.teams[TEAM_RED].players[i].penalty = 0;
          control_data.teams[TEAM_RED].players[i].secsTillUnpenalised = 0;
          remove_penalty(get_red_robot_index(i));
        }
      }
      if (robots[get_blue_robot_index(i)] && control_data.teams[TEAM_BLUE].players[i].secsTillUnpenalised > 0) {
        control_data.teams[TEAM_BLUE].players[i].secsTillUnpenalised -= TIME_STEP / 1000.0;
        if (control_data.teams[TEAM_BLUE].players[i].secsTillUnpenalised <= 0) {
          control_data.teams[TEAM_BLUE].players[i].penalty = 0;
          control_data.teams[TEAM_BLUE].players[i].secsTillUnpenalised = 0;
          remove_penalty(get_blue_robot_index(i));
        }
      }
    }

    if (kick_off_state == KICK_OFF_INITIAL) {
      update_kick_off_state(startingTime, timeRemaining);

      // "If a defensive player enters the center circle before the ball
      // is in play, the “Illegal Defender” penalty is applied"
      for (i = 0; i < MAX_NUM_PLAYERS; ++i) {
        if (control_data.kickOffTeam == TEAM_BLUE && robots[get_red_robot_index(i)]) {
          if (is_robot_in_central_circle(get_red_robot_index(i))) {
            char msg[] = "Illegal Defender (Red X)";
            msg[22] = (char)(((int)'0') + i + 1);
            show_message(msg);
            control_data.teams[TEAM_RED].players[i].penalty = PENALTY_ILLEGAL_DEFENDER;
            control_data.teams[TEAM_RED].players[i].secsTillUnpenalised = PENALTY_TIME;
            penalize_robot(get_red_robot_index(i));
          }
        } else if (control_data.kickOffTeam == TEAM_RED && robots[get_blue_robot_index(i)]) {
          if (is_robot_in_central_circle(get_blue_robot_index(i))) {
            char msg[] = "Illegal Defender (Blue X)";
            msg[23] = (char)(((int)'0') + i + 1);
            show_message(msg);
            control_data.teams[TEAM_BLUE].players[i].penalty = PENALTY_ILLEGAL_DEFENDER;
            control_data.teams[TEAM_BLUE].players[i].secsTillUnpenalised = PENALTY_TIME;
            penalize_robot(get_blue_robot_index(i));
          }
        }
      }
    }

    check_ball_out();

    check_position_defenders();

    if (is_ball_in_red_goal()) {  // ball in the red goal
      // a goal cannot be scored directly from a kick-off
      if (control_data.kickOffTeam == TEAM_RED || kick_off_state == KICK_OFF_PLAYED) {
        control_data.teams[TEAM_BLUE].score++;
        show_message("GOAL!");
      } else
        show_message("KICK-OFF SHOT!");

      control_data.state = STATE_READY;
      control_data.kickOffTeam = TEAM_RED;
      return;
    } else if (is_ball_in_blue_goal()) {  // ball in the blue goal
      // a goal cannot be scored directly from a kick-off
      if (control_data.kickOffTeam == TEAM_BLUE || kick_off_state == KICK_OFF_PLAYED) {
        control_data.teams[TEAM_RED].score++;
        show_message("GOAL!");
      } else
        show_message("KICK-OFF SHOT!");

      control_data.state = STATE_READY;
      control_data.kickOffTeam = TEAM_BLUE;
      return;
    }

    step();
  }
}

static void terminate() {
  if (match_type != DEMO) {
    FILE *file = fopen("scores.txt", "w");
    if (file) {
      fprintf(file, "%d\n%d\n", control_data.teams[TEAM_RED].score, control_data.teams[TEAM_BLUE].score);
      fclose(file);
    } else
      fprintf(stderr, "could not write: scores.txt\n");

    // give some time to show scores
    run_seconds(10);

    // freeze webcam
    int ret = system("killall webcam.php");
    if (ret != 0)
      fprintf(stderr, "Failed to kill webcam.php\n");

    // terminate movie recording and quit
    wb_supervisor_movie_stop_recording();
    wb_robot_step(0);
    wb_supervisor_simulation_quit(EXIT_SUCCESS);
  }

  int i;
  for (i = 0; i < MAX_NUM_ROBOTS; ++i) {
#ifdef _WIN32
    closesocket(fd[i]);
#else
    close(fd[i]);
#endif
  }

  while (1)
    step();  // wait forever
}

static void run_half_time_break() {
  int i;
  show_message("HALF TIME BREAK!");

  // remove penalties at half-time
  for (i = 0; i < MAX_NUM_PLAYERS; ++i) {
    control_data.teams[TEAM_RED].players[i].penalty = 0;
    control_data.teams[TEAM_RED].players[i].secsTillUnpenalised = 0;
    control_data.teams[TEAM_BLUE].players[i].penalty = 0;
    control_data.teams[TEAM_BLUE].players[i].secsTillUnpenalised = 0;
  }
  step();
}

static void run_half_time() {
  // first kick-off of the half is always for the reds
  if (control_data.firstHalf)
    control_data.kickOffTeam = TEAM_RED;
  else
    control_data.kickOffTeam = TEAM_BLUE;

  run_initial_state();

  do {
    run_ready_state();
    run_set_state();
    run_playing_state();
  } while (control_data.state != STATE_FINISHED);

  run_finished_state();
}

// randomize initial position for penalty kick shootout
static double randomize_pos() {
  return (double)rand() / (double)RAND_MAX * 0.01 - 0.005;  // +/- 1 cm
}

// randomize initial angle for penalty kick shootout
static double randomize_angle() {
  return (double)rand() / (double)RAND_MAX * 0.1745 - 0.0873;  // +/- 5 degrees
}

static int ball_completely_outside_penalty_area() {
  const double X_LIMIT = FIELD_SIZE_X / 2 - PENALTY_AREA_X_DIM - BALL_RADIUS;
  const double Y_LIMIT = PENALTY_AREA_Y_DIM / 2 + BALL_RADIUS;
  return ball_pos[X] > -X_LIMIT || fabs(ball_pos[Y]) > Y_LIMIT;
}

static void run_penalty_kick(double delay, int team_color) {
  // game control
  timeRemaining = delay;
  control_data.kickOffTeam = team_color;
  control_data.state = STATE_SET;
  display();

  // "The ball is placed on the penalty spot"
  move_ball_2d(-PENALTY_BALL_X_POS + randomize_pos(), randomize_pos());

  int attacker;
  int goalie;

  // attacker and goalie indices during penalties
  if (team_color) {
    attacker = get_blue_robot_index(ATTACKER);
    goalie = get_red_robot_index(GOALIE);
  } else {
    attacker = get_red_robot_index(ATTACKER);
    goalie = get_blue_robot_index(GOALIE);
  }

  // move other robots out of the soccer field
  int i, j = 0;
  for (i = 0; i < MAX_NUM_ROBOTS; i++) {
    if (robots[i] && i != attacker && i != goalie) {
      // preserve elevation to avoid dropping them or putting them through the floor
      double elevation = wb_supervisor_field_get_sf_vec3f(robots[i]->translation)[Y];
      double out_of_field[3] = {10.0, elevation, ((0 < i - MAX_NUM_PLAYERS) - (i - MAX_NUM_PLAYERS < 0)) * (5.0 + 2 * j++)};
      wb_supervisor_field_set_sf_vec3f(robots[i]->translation, out_of_field);
    }
  }

  // "The attacking robot is positioned 1 meter behind the penalty spot, facing the ball"
  // "The goal keeper is placed with feet on the goal line and in the centre of the goal"
  const double ATTACKER_POS[3] = {-PENALTY_BALL_X_POS + 1 + randomize_pos(), randomize_pos(), M_PI + randomize_angle()};
  const double GOALIE_POS[3] = {-PENALTY_GOALIE_X_POS + randomize_pos(), randomize_pos(), randomize_angle()};

  // hold attacker and goalie for 1.5 seconds in place during the SET state
  int n;
  for (n = 1500 / TIME_STEP; n > 0; n--) {
    move_robot_2d(attacker, ATTACKER_POS[0], ATTACKER_POS[1], ATTACKER_POS[2]);
    move_robot_2d(goalie, GOALIE_POS[0], GOALIE_POS[1], GOALIE_POS[2]);
    step();
  }

  // switch to PLAYING state
  control_data.state = STATE_PLAYING;

  if (team_color) {
    scoredBlue = 0;
    touchedBlue = 1;
    timeSpentBlue = 0.0;
    marginOfErrorBlue = 5.0;
  } else {
    scoredRed = 0;
    touchedRed = 1;
    timeSpentRed = 0.0;
    marginOfErrorRed = 5.0;
  }

  int alreadyTouched = 0;
  int nbStepsBallNotMoving = 0;
  double xBall = 0.0;
  double yBall = 0.0;
  do {
    // substract TIME_STEP to current time
    timeRemaining -= TIME_STEP / 1000.0;
    display();

    if (timeRemaining < 0.0) {
      timeRemaining = 0.0;
      show_message("TIME OUT!");
      if (team_color)
        touchedBlue = 0;
      else
        touchedRed = 0;
      return;
    }

    int robot_index = detect_ball_touch();

    if (alreadyTouched > 1)
      --alreadyTouched;

    if (alreadyTouched && (nbStepsBallNotMoving < 10)) {
      if (!nbStepsBallNotMoving) {
        xBall = ball_pos[X];
        yBall = ball_pos[Y];
        ++nbStepsBallNotMoving;
      } else if ((ball_pos[X] == xBall) && (ball_pos[Y] == yBall)) {
        ++nbStepsBallNotMoving;
      } else {
        nbStepsBallNotMoving = 0;
      }
    }

    // "The attacking robot is only allowed to contact the ball once"
    if ((robot_index == attacker) && (alreadyTouched == 1)) {
      xBall = ball_pos[X];
      yBall = ball_pos[Y];
      double xMargin = fabs(xBall + FIELD_X_LIMIT);
      double yMargin;
      if (fabs(yBall) > (GOAL_WIDTH / 2))
        yMargin = fabs(yBall) - (GOAL_WIDTH / 2);
      else
        yMargin = 0;
      if (team_color)
        marginOfErrorBlue = sqrt((xMargin * xMargin) + (yMargin * yMargin));
      else
        marginOfErrorRed = sqrt((xMargin * xMargin) + (yMargin * yMargin));
      show_message("ILLEGAL ACTION! (2ND TOUCH)");
      return;
    }

    // "Hence, a penalty shot is over when the ball has come to a full stop after
    // the first contact by the attacking robot"
    if ((nbStepsBallNotMoving == 10) && (alreadyTouched == 1)) {
      xBall = ball_pos[X];
      yBall = ball_pos[Y];
      double xMargin = fabs(xBall + FIELD_X_LIMIT);
      double yMargin;
      if (fabs(yBall) > (GOAL_WIDTH / 2))
        yMargin = fabs(yBall) - (GOAL_WIDTH / 2);
      else
        yMargin = 0;
      if (team_color)
        marginOfErrorBlue = sqrt((xMargin * xMargin) + (yMargin * yMargin));
      else
        marginOfErrorRed = sqrt((xMargin * xMargin) + (yMargin * yMargin));
      show_message("BALL NOT IN!");
      return;
    }

    if ((robot_index == attacker) && !alreadyTouched)
      alreadyTouched = 10;  // don't put it immediately to 1 in case the contact with the ball lasts more than one time step

    // "If the goal keeper touches the ball outside the penalty area then a goal will be awarded to the attacking team"
    if (robot_index == goalie && ball_completely_outside_penalty_area()) {
      if (team_color) {
        control_data.teams[TEAM_BLUE].score++;
        scoredBlue = 1;
        timeSpentBlue = delay - timeRemaining;
      } else {
        control_data.teams[TEAM_RED].score++;
        scoredRed = 1;
        timeSpentRed = delay - timeRemaining;
      }
      show_message("ILLEGAL GOALIE ACTION!");
      return;
    }

    step();
  } while (is_ball_in_field());

  if (is_ball_in_blue_goal()) {
    if (team_color) {
      control_data.teams[TEAM_BLUE].score++;
      timeSpentBlue = delay - timeRemaining;
      scoredBlue = 1;
    } else {
      control_data.teams[TEAM_RED].score++;
      timeSpentRed = delay - timeRemaining;
      scoredRed = 1;
    }
    show_message("GOAL!");
  } else {
    xBall = ball_pos[X];
    yBall = ball_pos[Y];
    double xMargin = fabs(xBall + FIELD_X_LIMIT);
    double yMargin;
    if (fabs(yBall) > (GOAL_WIDTH / 2))
      yMargin = fabs(yBall) - (GOAL_WIDTH / 2);
    else
      yMargin = 0;
    if (team_color)
      marginOfErrorBlue = sqrt((xMargin * xMargin) + (yMargin * yMargin));
    else
      marginOfErrorRed = sqrt((xMargin * xMargin) + (yMargin * yMargin));
    show_message("MISSED!");
  }
}

static void run_victory(int team_color) {
  char winMessage[128];
  sprintf(winMessage, "%s WINS!", get_team_name(team_color));
  show_message(winMessage);
  run_finished_state();
}

// "A winner can be declared before the conclusion of the penalty shoot-out
// if a team can no longer win, {...}"
static int check_victory(int remaining_attempts) {
  int diff = control_data.teams[TEAM_RED].score - control_data.teams[TEAM_BLUE].score;
  if (diff > remaining_attempts) {
    run_victory(TEAM_RED);
    return 1;
  } else if (diff < -remaining_attempts) {
    run_victory(TEAM_BLUE);
    return 1;
  }
  return 0;
}

static void run_penalty_kick_shootout() {
  show_message("PENALTY KICK SHOOT-OUT!");
  step();

  // inform robots of the penalty kick shootout
  control_data.secondaryState = STATE2_PENALTYSHOOT;

  // five penalty shots per team
  int i;
  for (i = 0; i < 5; i++) {
    run_penalty_kick(60, TEAM_RED);
    if (check_victory(5 - i))
      return;
    run_finished_state();
    run_penalty_kick(60, TEAM_BLUE);
    if (check_victory(4 - i))
      return;
    run_finished_state();
  }

  if (match_type == FINAL) {
    show_message("SUDDEN DEATH SHOOT-OUT!");

    // sudden death shots
    while (1) {
      run_penalty_kick(120, TEAM_RED);
      run_finished_state();
      run_penalty_kick(120, TEAM_BLUE);
      if (check_victory(0)) {
        return;
      } else if (scoredRed && scoredBlue) {
        if ((timeSpentRed + 2) <= timeSpentBlue) {
          printf("The red team scored %.2f seconds faster than the blue team.\n", timeSpentBlue - timeSpentRed);
          run_victory(TEAM_RED);
          return;
        } else if ((timeSpentBlue + 2) <= timeSpentRed) {
          printf("The blue team scored %.2f seconds faster than the red team.\n", timeSpentRed - timeSpentBlue);
          run_victory(TEAM_BLUE);
          return;
        }
        printf("The two teams scored in %f (red) and %f (blue) seconds, shoot again.\n", timeSpentRed, timeSpentBlue);
      } else if (!(touchedRed || touchedBlue)) {
        printf("Both team didn't touch the ball, selecting a winner at random...\n");
        srand(time(NULL));
        int winner = rand() % 2;
        run_victory(winner);
        return;
      } else {
        if ((marginOfErrorRed + 0.05) < marginOfErrorBlue) {
          printf("The red team shot the ball %dmm closer to the goal than the blue team.\n",
                 (int)(1000 * (marginOfErrorBlue - marginOfErrorRed)));
          run_victory(TEAM_RED);
          return;
        } else if ((marginOfErrorBlue + 0.05) < marginOfErrorRed) {
          printf("The blue team shot the ball %dmm closer to the goal than the red team.\n",
                 (int)(1000 * (marginOfErrorRed - marginOfErrorBlue)));
          run_victory(TEAM_BLUE);
          return;
        }
        printf("The two teams both shot the ball nearly as close to the goal as the other, shoot again.\n");
      }
      run_finished_state();
    }
  }
}

int main(int argc, const char *argv[]) {
  // init devices and data structures
  initialize();

  // check controllerArgs
  if (argc > 1 && strcmp(argv[1], "penalty") == 0)
    // run only penalty kicks
    run_penalty_kick_shootout();
  else {
    // first half-time
    control_data.firstHalf = 1;
    run_half_time();
    run_half_time_break();

    // second half-time
    control_data.firstHalf = 0;
    run_half_time();

    // if the game is tied, start penalty kicks
    if (control_data.teams[TEAM_BLUE].score == control_data.teams[TEAM_RED].score)
      run_penalty_kick_shootout();
  }

  // terminate movie, write scores.txt, etc.
  terminate();

  return 0;  // never reached
}
