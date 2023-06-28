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
 * Description:  A simple program implementing a Unix pipe relay controller
 *               for interfacing Webots with any development environment able
 *               to use pipes, like Lisp.
 */

/*
 * The protocole used in this example is taken from the Khepera serial
 * communication protocole. Hence, if you already have developed an
 * application which uses this protocole (and send the data over the serial
 * port), you will just need to redirect the data to the pipe file of
 * this controller to make it work with Webots.
 *
 * A sample client program, written in C is included in this directory.
 * See client.c for the source code
 *
 * Everything relies on standard unix pipes.
 */

#ifdef _WIN32 /* not working on Windows */
int main() {
  return 0;
}
#else
// clang-format off
// clang-format 11.0.0 has problems with AlignTrailingComments when the line starts with a hash (#).
#include <fcntl.h>     // definition of O_RDONLY and O_WRONLY
#include <stdio.h>     // definition of sprintf, fprintf, sscanf and stderr
#include <stdlib.h>    // definition of exit()
#include <string.h>    // definition of strcpy() and strlen()
#include <sys/stat.h>  // definition of mknod() and S_IFIFO
#include <unistd.h>    // definition of close()
// clang-format on
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include "common.h"

static int sim_serial_in, sim_serial_out;
static WbDeviceTag ds[NB_IR_SENSOR], ls[NB_IR_SENSOR], left_motor, right_motor, left_position_sensor, right_position_sensor;

static void open_pipes() {
  mknod(PIPE_FILE_DIRECTORY ".sim_serial_in", S_IFIFO | 0666, 0);
  mknod(PIPE_FILE_DIRECTORY ".sim_serial_out", S_IFIFO | 0666, 0);

  do {
    if (sim_serial_in == -1)
      sim_serial_in = open(PIPE_FILE_DIRECTORY ".sim_serial_in", O_RDONLY, 0);
    if (sim_serial_out == -1)
      sim_serial_out = open(PIPE_FILE_DIRECTORY ".sim_serial_out", O_WRONLY, 0);
    if (sim_serial_in == -1 || sim_serial_out == -1) {
      fprintf(stderr,
              "pipe controller: cannot open pipes: %d %d, "
              "keep trying\n",
              sim_serial_in, sim_serial_out);
      sleep(1);
    }
  } while (sim_serial_out == -1 || sim_serial_in == -1);
}

static void close_pipes() {
  close(sim_serial_in);
  close(sim_serial_out);

  unlink(PIPE_FILE_DIRECTORY ".sim_serial_in");
  unlink(PIPE_FILE_DIRECTORY ".sim_serial_out");
}

int main() {
  wb_robot_init();  // initialize webots

  char a[256], q[256];
  int left_speed, right_speed;
  unsigned short ds_value[NB_IR_SENSOR], ls_value[NB_IR_SENSOR];
  int n;

  sim_serial_in = -1;
  sim_serial_out = -1;

  open_pipes();

  int i;
  char text[32];
  for (i = 0; i < NB_IR_SENSOR; i++) {
    sprintf(text, "ds%d", i);
    ds[i] = wb_robot_get_device(text);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
    sprintf(text, "ls%d", i);
    ls[i] = wb_robot_get_device(text);
    wb_light_sensor_enable(ls[i], TIME_STEP);
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    for (i = 0; i < NB_IR_SENSOR; i++) {
      ds_value[i] = wb_distance_sensor_get_value(ds[i]);
      ls_value[i] = wb_light_sensor_get_value(ls[i]);
    }

    const int left_encoder = wb_position_sensor_get_value(left_position_sensor);
    const int right_encoder = wb_position_sensor_get_value(right_position_sensor);

    /* These are all the commands recognized by this controller. */
    a[0] = '\0';
    if (read(sim_serial_in, &q[0], 1) != 0) {
      switch (q[0]) {
        case 'A':
          fprintf(stderr, "Warning: serial command A (Configure) "
                          "not implemented");
          strcpy(a, "a\n");
          break;
        case 'B':
          strcpy(a, "b,0,0\n"); /* 0,0 means simulation */
          break;
        case 'D':
          i = 0;
          do {
            i++;
            n = read(sim_serial_in, &q[i], 1);
            if (n < 0)
              fprintf(stderr, "Warning: an error occured when reading the pipe");
          } while (q[i] != '\n');
          sscanf(q, "D,%d,%d", &left_speed, &right_speed);
          wb_motor_set_velocity(left_motor, left_speed);
          wb_motor_set_velocity(right_motor, right_speed);
          strcpy(a, "d\n");
          break;
        case 'E':
          sprintf(a, "e,%d,%d\n", left_speed, right_speed);
          break;
        case 'G':
          fprintf(stderr, "Warning: serial command G (Set position) not implemented");
          strcpy(a, "g\n");
          break;
        case 'H':
          sprintf(a, "h,%d,%d\n", left_encoder, right_encoder);
          break;
        case 'N':
          sprintf(a, "n,%d,%d,%d,%d,%d,%d,%d,%d\n", ds_value[0], ds_value[1], ds_value[2], ds_value[3], ds_value[4],
                  ds_value[5], ds_value[6], ds_value[7]);
          break;
        case 'O':
          sprintf(a, "o,%d,%d,%d,%d,%d,%d,%d,%d\n", ls_value[0], ls_value[1], ls_value[2], ls_value[3], ls_value[4],
                  ls_value[5], ls_value[6], ls_value[7]);
          break;
      }
    }
    if (a[0] != '\0') {
      n = write(sim_serial_out, a, strlen(a));
      if (n < 0)
        fprintf(stderr, "Warning: an error occured when reading the pipe");
    }
  }

  close_pipes();
  wb_robot_cleanup();

  return 0;
}
#endif /* _WIN32 */
