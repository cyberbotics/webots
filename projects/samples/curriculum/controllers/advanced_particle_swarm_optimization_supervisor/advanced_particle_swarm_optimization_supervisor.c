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
 * Description:   Supervisor of the advanced exercise on PSO
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

/* Simulation parameters */
#define NB_RUN 42          // TODO: won't compile // the number of run
#define END_TIME 1337      // TODO: won't compile // the duration of each run [us]
#define NOISE_RESISTANT 0  // enables the node resistant version of PSO

#define TIME_STEP 64
#define NB_ROBOTS 10
#define ARENA_SIZE 1.45  // this is the size of the side of the square area [m]
#define TYPES SUPERVISOR_FIELD_TRANSLATION | SUPERVISOR_FIELD_ROTATION_ANGLE
#define COMMUNICATION_CHANNEL 1

typedef enum {
  STOP_RUN = 0,  // no parameter
  RESET,         // no parameter
  MY_FITNESS,    // with robot_id,fitness
  SAVE_GBEST,    // no parameter
  DEMO,          // no parameter
  EVAL_PBEST     // no parameter
} MessageType;

/*
 * Global variables
 */
WbDeviceTag emitter;
WbDeviceTag receiver;
WbNodeRef robots[NB_ROBOTS];
WbFieldRef trans_field[NB_ROBOTS];

float gbest = 0;

static void reset(void);
static void random_position(void);
static void step();

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

/*
 * This is the reset function called once upon initialization of the robot.
 */
static void reset(void) {
  srand(time(NULL));

  // Get emitter and receiver devices.
  emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL);
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  // Name of the robot we are linking.
  char rob[7] = "epuck0";
  int i = 0;
  for (; i < NB_ROBOTS; i++) {
    // Indicate where robot location has to be stored.
    robots[i] = wb_supervisor_node_get_from_def(rob);
    trans_field[i] = wb_supervisor_node_get_field(robots[i], "translation");

    // Update robot name.
    rob[5]++;
  }

  return;
}

/*
 * Places each robot randomly in the arena
 */
static void random_position(void) {
  double p[3] = {0, 0, 0};
  int n = 0;
  for (; n < NB_ROBOTS; n++) {
    p[0] = (ARENA_SIZE * 0.8) * ((double)rand() / RAND_MAX) - ARENA_SIZE / 2.0;  // x coordinate
    p[1] = (ARENA_SIZE * 0.8) * ((double)rand() / RAND_MAX) - ARENA_SIZE / 2.0;  // y coordinate

    wb_supervisor_field_set_sf_vec3f(trans_field[n], p);
  }
  step();
  printf("Random positioning done.\n");
  return;
}

/*
 * This is the main program which sets up the reset and run function.
 */
int main() {
  wb_robot_init();

  int i;
  float f;
  float outbuffer[2];

  reset();

  printf("Start the simulation\n");
  int run_counter = 0;

  for (; run_counter < NB_RUN; run_counter++) {
    printf("Start of run #%d\n", run_counter + 1);
    random_position();  // randomize robot positions

    // main simulation loop
    int time_cpt = 0;
    for (; time_cpt < END_TIME; time_cpt += TIME_STEP) {
      step();
    }

    float fitness_received[NB_ROBOTS];
    int nb_received = 0;
    for (i = 0; i < NB_ROBOTS; i++)
      fitness_received[i] = 0;

    // robot_console_printf("Receiver queue length: %d\n", receiver_get_queue_length(receiver));
    while (wb_receiver_get_queue_length(receiver) > 0) {
      wb_receiver_next_packet(receiver);
      // robot_console_printf("Receiver queue length: %d\n", receiver_get_queue_length(receiver));
    }

    // broadcast STOP_RUN message
    outbuffer[0] = -1;
    outbuffer[1] = STOP_RUN;
    wb_emitter_send(emitter, outbuffer, 2 * sizeof(float));
    step();

    while (nb_received != NB_ROBOTS) {
      // robot_console_printf("In the while, receiver queue length: %d\n", receiver_get_queue_length(receiver));
      float *inbuffer;
      // get all the new messages from the robots
      while (wb_receiver_get_queue_length(receiver) > 0) {
        // robot_console_printf("Something received\n");
        inbuffer = (float *)wb_receiver_get_data(receiver);  // get pointer to receiver buffer

        // if the message is for me, parse it
        if (inbuffer[0] == (float)(42)) {
          switch ((int)inbuffer[1]) {
            case MY_FITNESS:
              f = inbuffer[3];
              fitness_received[(int)inbuffer[2]] = f;
              break;
            default:
              break;
          }
        }

        wb_receiver_next_packet(receiver);
      }

      // check how many fitnesses were received
      nb_received = 0;
      for (i = 0; i < NB_ROBOTS; i++) {
        if (fitness_received[i] != (float)0)
          nb_received++;
        else {
          // send STOP_RUN message to the robot that didn't sent their fitness
          outbuffer[0] = i;
          outbuffer[1] = STOP_RUN;
          wb_emitter_send(emitter, outbuffer, 2 * sizeof(float));
          step();
        }
      }
    }

    // check if gbest was beaten and by which robot
    float max = 0;
    int whom = 0;
    for (i = 0; i < NB_ROBOTS; i++) {
      if (fitness_received[i] > max) {
        max = fitness_received[i];
        whom = i;
      }
    }
    // if yes => send SAVE_GBEST to the best robot
    if (max > gbest) {
      gbest = max;
      outbuffer[0] = whom;
      outbuffer[1] = SAVE_GBEST;
      wb_emitter_send(emitter, outbuffer, 2 * sizeof(float));
    }

    step();

    outbuffer[0] = -1;
    if (NOISE_RESISTANT == 1 && run_counter % 3 == 2) {
      gbest = 0;
      // broadcast EVAL_PBEST again
      outbuffer[1] = EVAL_PBEST;
    } else  // broadcast RESET message
      outbuffer[1] = RESET;
    wb_emitter_send(emitter, outbuffer, 2 * sizeof(float));

    step();

    printf("Fin du run #%d\n", run_counter + 1);
  }

  // end of the simulation, enter DEMO mode
  outbuffer[0] = -1;
  outbuffer[1] = DEMO;
  wb_emitter_send(emitter, outbuffer, 2 * sizeof(float));

  printf("End of the simulation, entering DEMO mode\n");
  random_position();  // randomize robot positions

  // stay in this mode forever
  while (true)
    step();

  return 0;
}
