/*
 * Copyright 1996-2019 Cyberbotics Ltd.
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
 * Description:  A controller running a race stopwatch.
 */

#include <stdio.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/utils/default_robot_window.h>
#include "../../../include/robotbenchmark.h"

#define N_DIGIT 6

static void stopwatch_init(WbDeviceTag digit[N_DIGIT][12]) {
  char digit_name[8];
  snprintf(digit_name, 8, "digit__");
  int i, j;
  for (i = 0; i < N_DIGIT; ++i) {
    digit_name[5] = '0' + i;
    for (j = 0; j < 12; ++j) {
      if (j == 10)
        digit_name[6] = 'x';
      else if (j == 11)
        digit_name[6] = '_';
      else
        digit_name[6] = '0' + j;
      digit[i][j] = wb_robot_get_device(digit_name);
      if (digit[i][j] == 0)
        fprintf(stderr, "missing %s device\n", digit_name);
    }
  }
}

static void stopwatch_set_time(WbDeviceTag digit[N_DIGIT][12], double t) {
  static int digit_value[N_DIGIT] = {0, 0, 0, 0, 0, 0};
  int i;
  for (i = 0; i < N_DIGIT; ++i)
    wb_motor_set_position(digit[i][digit_value[i]], 0);  // hide old digit
  digit_value[0] = (int)(t * 100) % 10;
  if (t >= 0.1) {
    digit_value[1] = (int)(t * 10) % 10;
    if (t >= 1) {
      digit_value[2] = (int)(t) % 10;
      if (t >= 10) {
        digit_value[3] = (int)(t / 10) % 6;
        if (t >= 60) {
          digit_value[4] = (int)(t / 60) % 10;
          if (t >= 600)
            digit_value[5] = (int)(t / 600) % 10;
        }
      }
    }
  }
  for (i = 0; i < N_DIGIT; ++i)
    wb_motor_set_position(digit[i][digit_value[i]], 0.1);  // show new digit
}

int main(int argc, char *argv[]) {
  WbDeviceTag digit[N_DIGIT][12];
  wb_robot_init();
  stopwatch_init(digit);
  double time_step = wb_robot_get_basic_time_step();
  WbDeviceTag detector = wb_robot_get_device("detector");
  wb_distance_sensor_enable(detector, time_step);
  WbNodeRef nao = wb_supervisor_node_get_from_def("NAO");
  WbFieldRef translation = wb_supervisor_node_get_field(nao, "translation");
  int run = 1;
  double record = 0;
  double penalty = 0;
  bool has_penality = false;
  do {
    const double *v = wb_supervisor_field_get_sf_vec3f(translation);
    if (!has_penality && v[1] < 0.2) {  // the robot has fallen down
      printf("The robot is down, a penalty of 30 seconds is added.\n");
      penalty += 30;
      has_penality = true;
    }
    double t = wb_robot_get_time() + penalty;
    if (run) {
      stopwatch_set_time(digit, t + time_step / 1000);  // to avoid discrepancy with Webots time
      char buffer[32];
      snprintf(buffer, 32, "time:%-24.3f", t);
      int i;
      for (i = 5; i < sizeof(buffer); i++)
        if (buffer[i] == ' ') {
          buffer[i] = '\0';
          break;
        }
      wb_robot_wwi_send_text(buffer);
    }
    if (wb_distance_sensor_get_value(detector) < 2.4) {
      if (run) {
        record = t;
        int m = t / 60;
        double s = t - 60 * m;
        printf("Time is: %d.%05.2f\n", m, s);
        stopwatch_set_time(digit, t);  // display actual time
        wb_robot_wwi_send_text("stop");
      }
      run = 0;
    }
    const char *message = wb_robot_wwi_receive_text();
    if (message && strncmp(message, "record:", 7) == 0)
      // because the smallest record is the best, we send a negative value here
      robotbenchmark_record(message, "humanoid_sprint", -record);
  } while (wb_robot_step(time_step) != -1);
  wb_robot_cleanup();
  return 0;
}
