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
 * Description:   Controller of the advanced exercise on PSO
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#include "ann.h"  // definition of the used ANN
#include "pso.h"  // definitions of the ANN functions

#define TIME_STEP 64
#define NB_SENSORS 8
#define MAX_IR_VALUE 3800
#define COMMUNICATION_CHANNEL 1
#define SPEED_UNIT 0.00628

typedef enum {
  STOP_RUN = 0,  // no parameter
  RESET,         // no parameter
  MY_FITNESS,    // with robot_id,fitness
  SAVE_GBEST,    // no parameter
  DEMO,          // no parameter
  EVAL_PBEST     // no parameter
} MessageType;

typedef enum { RUN = 0, STOP, WAIT_RESET, PBEST, ROBOT_DEMO } RobotState;

static void reset(void);
static int run(void);
static void state_run(void);
static void state_stop(void);
static void state_demo(void);
static void evolve_and_reset(void);
static float f_abs(float);
static float get_fitness(void);

WbDeviceTag ds[NB_SENSORS];           // Handle for the infrared distance sensors
WbDeviceTag emitter;                  // Handle for the emitter node
WbDeviceTag receiver;                 // Handle for the receiver node
WbDeviceTag left_motor, right_motor;  // Handle for the Motor nodes

RobotState state = RUN;
int robot_id;  // Unique robot ID
float distances[NB_SENSORS];
float bias[2] = {1, 1};
float prev_speed[2] = {0, 0};

// variables used to compute the fitness function
float f_V = 0;
float f_dv = 0;
float f_i = 0;
int f_cpt = 0;

float pbest = 0;

// absolute value on floats
static float f_abs(float x) {
  return x < 0 ? -x : x;
}

// compute the fitness function
static float get_fitness() {
  /* TODO : compute the fitness function and return it.
   * You can use the variables f_V f_dv and f_i which are the V, dv and i
   * of the proposed fitness function. f_cpt is the number of time those values
   * were updated (useful for the average.
   */
  return 0.0;
}

/*
 * This is the reset function called once upon initialization of the robot.
 */
static void reset(void) {
  // Get emitter and receiver devices.
  emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL);
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  // Get a handler to the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  char s[4] = "ps0";
  int i = 0;
  for (; i < NB_SENSORS; i++) {
    ds[i] = wb_robot_get_device(s);  // the device name is specified in the world file
    s[2]++;                          // increases the device number
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  char *robot_name;
  robot_name = (char *)wb_robot_get_name();

  sscanf(robot_name, "epuck%d", &robot_id);  // read robot id from the robot's name

  printf("Reset: robot %d\n", robot_id);

  state = RUN;  // begin in state RUN
  prev_speed[0] = 0;
  prev_speed[1] = 0;
  f_V = 0;
  f_dv = 0;
  f_i = 0;
  f_cpt = 0;

  return;
}

/*
 * This function is called when the state is RUN
 */
static void state_run(void) {
  int i = 0;
  float values[NB_SENSORS + 4];
  float tmp = 0;
  // get distance sensors values
  for (; i < NB_SENSORS; i++) {
    values[i] = (float)wb_distance_sensor_get_value(ds[i]);
    tmp = values[i] > tmp ? values[i] : tmp;
  }

  values[NB_SENSORS] = bias[0];
  values[NB_SENSORS + 1] = bias[1];
  values[NB_SENSORS + 2] = prev_speed[0];
  values[NB_SENSORS + 3] = prev_speed[1];

  InputToNetwork(&network, values);         // feed the ANN with the sensors values + bias + previous speed
  ActivateNetwork(&network);                // activate the ANN
  OutputFromNetwork(&network, prev_speed);  // get the output from the ANN

  // set robot speed according to the ANN output
  wb_motor_set_velocity(left_motor, SPEED_UNIT * (2000 * prev_speed[0] - 1000));
  wb_motor_set_velocity(right_motor, SPEED_UNIT * (2000 * prev_speed[1] - 1000));

  // update variables for the fitness computation
  f_V += f_abs(prev_speed[0] + prev_speed[1]) / 2;
  f_dv += f_abs(prev_speed[0] - prev_speed[1]);
  f_i += tmp / MAX_IR_VALUE;
  f_cpt++;

  return;
}

/*
 * This function is called when the state is STOP
 */
static void state_stop(void) {
  // save current weights
  char filename[5] = "w0.w";
  filename[1] += robot_id;
  SaveNetworkWeights(&network, filename);

  // compute fitness function
  float fitness = get_fitness();

  // replace personal best if we reach a higher fitness
  if (fitness > pbest) {
    pbest = fitness;
    filename[0] = 'p';
    SaveNetworkWeights(&network, filename);
  }

  // send the current fitness to supervisor
  float outbuffer[4] = {42, MY_FITNESS, robot_id, fitness};
  wb_emitter_send(emitter, outbuffer, 4 * sizeof(float));

  printf("Fitness sent to supervisor : %f\n", fitness);

  // wait for reset and next run
  state = WAIT_RESET;
  return;
}

/*
 * This function is called to demonstrate the results of the evolution
 */
static void state_demo(void) {
  int i = 0;
  float values[NB_SENSORS + 4];
  float tmp = 0;
  // get distance sensors values
  for (; i < NB_SENSORS; i++) {
    values[i] = (float)wb_distance_sensor_get_value(ds[i]);
    tmp = values[i] > tmp ? values[i] : tmp;
  }

  values[NB_SENSORS] = bias[0];
  values[NB_SENSORS + 1] = bias[1];
  values[NB_SENSORS + 2] = prev_speed[0];
  values[NB_SENSORS + 3] = prev_speed[1];

  InputToNetwork(&network, values);         // feed the network
  ActivateNetwork(&network);                // activate the ANN
  OutputFromNetwork(&network, prev_speed);  // get the output from the ANN

  // set robot speed
  wb_motor_set_velocity(left_motor, SPEED_UNIT * (2000 * prev_speed[0] - 1000));
  wb_motor_set_velocity(right_motor, SPEED_UNIT * (2000 * prev_speed[1] - 1000));

  // update variables for the fitness function
  f_V += f_abs(prev_speed[0] + prev_speed[1]) / 2;
  f_dv += f_abs(prev_speed[0] - prev_speed[1]);
  f_i += tmp / MAX_IR_VALUE;
  f_cpt++;

  // show current fitness once every 100 iteration
  if (f_cpt % 100 == 0)
    printf("Current fitness is %f\n", get_fitness());

  return;
}

/*
 * Evolve the ANN according to current particle, personal best and global best.
 * Then reset the robot for a new run.
 */
static void evolve_and_reset(void) {
  char filename[5] = "p0.w";
  filename[1] += robot_id;
  LoadNetworkWeights(&pnetwork, filename);

  LoadNetworkWeights(&gnetwork, "gb.w");

  EvolveNetwork(&network, &pnetwork, &gnetwork);

  reset();
}

/*
 * This is the main control loop function, it is called repeatedly by Webots
 */
static int run(void) {
  // get all the new messages from the supervisor
  while (wb_receiver_get_queue_length(receiver) > 0) {
    const float *inbuffer = (float *)wb_receiver_get_data(receiver);  // get pointer to receiver buffer

    // if the message is for me, parse it
    if (inbuffer[0] == (float)(-1) || inbuffer[0] == (float)robot_id) {
      switch ((int)inbuffer[1]) {
        case STOP_RUN:
          if (state == PBEST && get_fitness() < pbest)
            pbest = get_fitness();
          state = STOP;
          break;
        case RESET:
          evolve_and_reset();
          break;
        case SAVE_GBEST:
          printf("Message received: SAVE_GBEST\n");
          if (state == WAIT_RESET) {
            SaveNetworkWeights(&network, "gb.w");
            SaveNetworkWeightsHDT(&network, "gbHDT.w");
          }
          break;
        case DEMO:
          LoadNetworkWeights(&network, "gb.w");
          state = ROBOT_DEMO;
          break;
        case EVAL_PBEST:
          printf("Message received: EVAL_PBEST\n");
          state = PBEST;
          // load personal best in ANN
          char filename[5] = "p0.w";
          filename[1] += robot_id;
          LoadNetworkWeights(&network, filename);
          break;
        default:
          break;
      }
    }

    wb_receiver_next_packet(receiver);
  }

  // state machine
  switch (state) {
    case RUN:
      state_run();
      break;
    case STOP:
      state_stop();
      break;
    case WAIT_RESET:
      break;
    case ROBOT_DEMO:
      state_demo();
      break;
    case PBEST:
      state_run();
      break;
    default:
      break;
  }

  return TIME_STEP; /* this is the time step value, in milliseconds. */
}

/*
 * This is the main program which sets up the reset and run function.
 */
int main() {
  wb_robot_init();

  reset();

  srand(time(NULL) + 100 * robot_id);  // initialize the random number generator
  RandomizeNetwork(&network);          // initiate the ANN with random numbers

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1)
    run();

  wb_robot_cleanup();

  return 0;
}
