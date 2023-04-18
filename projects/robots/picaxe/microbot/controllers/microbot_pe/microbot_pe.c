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

#ifndef _WIN32  // not supported on Linux and macOS
int main() {
  return 1;
}
#else

#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include <stdio.h>
#include <windows.h>

#define NAMED_PIPE_NAME "\\\\.\\Pipe\\picaxe-webots"

static HANDLE pipe_handle = NULL;

static int pinsB = 0;
static int pinsC = 0;
static int adcC2 = 0;

static int update_required = 0;
static int previous_pinsC = 0;
static int previous_adcC2 = 0;

static int number_of_updates = 0;
static int simulation_delay = 0;

static WbDeviceTag left_bumper = 0;
static WbDeviceTag right_bumper = 0;
static WbDeviceTag floor_sensor = 0;
static WbDeviceTag left_led = 0;
static WbDeviceTag right_led = 0;
static WbDeviceTag push_button = 0;
static WbDeviceTag left_motor = 0;
static WbDeviceTag right_motor = 0;

static char simulation_type[64];
static char simulation_state[8];
static int simulation_pin_size = 0;

static void handle_message_line(const char *line) {
  if (strncmp(line, "number_of_updates:", 18) == 0) {
    sscanf(&line[18], "%d", &number_of_updates);
    return;
  } else if (strncasecmp(line, "SimulationDelay:", 16) == 0) {
    // sscanf(&line[16],"%d",&ignored); // ignored
  } else if (strncasecmp(line, "SimulationState:", 16) == 0) {
    sscanf(&line[16], "%7s", simulation_state);
    printf("%s\n", simulation_state);
  } else if (strncasecmp(line, "SimulationType:", 15) == 0) {
    sscanf(&line[15], "%63s", simulation_type);
    if (strcasecmp(simulation_type, "PICAXE-20X2") == 0)
      printf("PICAXE type: %s\n", simulation_type);
    else {
      fprintf(stderr, "ERROR: unsupported PICAXE type for microbot: %s\n", simulation_type);
      fprintf(stderr, "Please select 'PICAXE-20X2' in PICAXE Editor ");
      fprintf(stderr, "from the 'Settings' tab in the Workspace Explorer\n");
    }
  } else if (strncasecmp(line, "SimulationPinSize:", 18) == 0)
    sscanf(&line[18], "%d", &simulation_pin_size);  // unused
  else if (strncmp(line, "pinsB:", 6) == 0)
    sscanf(&line[6], "%d", &pinsB);

  number_of_updates--;
}

static void handle_message(const char *input_buffer) {
  int i, l = strlen(input_buffer);
  handle_message_line(input_buffer);  // set number_of_updates
  for (i = 0; i < l && number_of_updates > 0; i++)
    if (input_buffer[i] == '\n') {
      handle_message_line(&input_buffer[i + 1]);
      // printf("number of updates=%d\n",number_of_updates);
    }
  // printf("pinsB:%d simulation_delay:%d\n",pinsB,simulation_delay);
}

static const char *compute_update() {
  static char output_buffer[1024];

  snprintf(output_buffer, 1024, "number_of_updates:2\r\nadcC.2:%d\r\npinsC:%d\r\n", adcC2, pinsC);
  return output_buffer;
}

static void close_connection() {
  fprintf(stderr, "Lost connection with PICAXE Editor\n");
  CloseHandle(pipe_handle);
  pipe_handle = NULL;
  sprintf(simulation_state, "STOP");
  pinsB = 0;
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  wb_led_set(left_led, 0);
  wb_led_set(right_led, 0);
}

int main() {
  char input_buffer[1024];
  sprintf(simulation_state, "STOP");  // inially in STOP mode
  sprintf(simulation_type, "UNKNOWN");
  wb_robot_init();
  left_bumper = wb_robot_get_device("left bumper");
  right_bumper = wb_robot_get_device("right bumper");
  floor_sensor = wb_robot_get_device("floor sensor");
  left_led = wb_robot_get_device("left led");
  right_led = wb_robot_get_device("right led");
  push_button = wb_robot_get_device("push button");
  simulation_delay = wb_robot_get_basic_time_step();
  wb_touch_sensor_enable(left_bumper, simulation_delay);
  wb_touch_sensor_enable(right_bumper, simulation_delay);
  wb_touch_sensor_enable(push_button, simulation_delay);
  wb_distance_sensor_enable(floor_sensor, simulation_delay);
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  for (;;) {
    if (pipe_handle == NULL) {
      pipe_handle =
        CreateFile(NAMED_PIPE_NAME, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
      if (pipe_handle == INVALID_HANDLE_VALUE)
        pipe_handle = NULL;  // try again later
      else
        printf("Connected to PICAXE Editor\n");
    }
    if (pipe_handle) {
      DWORD e = NO_ERROR, nb_read = 0;
      if (!PeekNamedPipe(pipe_handle, NULL, 0, NULL, &nb_read, NULL)) {
        e = GetLastError();
        if (e == ERROR_BROKEN_PIPE) {
          close_connection();
          nb_read = 0;
        } else
          fprintf(stderr, "PeekNamedPipe error = %d\n", (int)e);
      }
      if (nb_read) {
        DWORD total_read = 0;
        BOOL success = false;
        do {
          nb_read = 0;
          success = ReadFile(pipe_handle, &input_buffer[total_read], sizeof(input_buffer) - total_read, &nb_read, NULL);
          total_read += nb_read;
          input_buffer[total_read] = '\0';
          // printf("received %d bytes: %s\n",strlen(input_buffer),input_buffer);
          if (!success)
            e = GetLastError();
        } while (!success && e == ERROR_MORE_DATA);
        if (success) {
          input_buffer[total_read] = '\0';
          handle_message(input_buffer);
        } else {
          if (e != ERROR_BROKEN_PIPE)
            fprintf(stderr, "Unknown error\n");
          close_connection();
        }
      }
    }
    if (pipe_handle && simulation_state[2] == 'A') {  // START state
      DWORD n;
      if (update_required == 1) {
        const char *update = compute_update();
        // printf("sending %s\n",update);
        if (update) {
          if (WriteFile(pipe_handle, update, strlen(update) * sizeof(char) + 1, &n, NULL) == 0) {
            if (GetLastError() != ERROR_BROKEN_PIPE)
              fprintf(stderr, "Unknown error\n");
            close_connection();
          }
          fflush(stdout);
        }
      }
    }
    int left_speed;
    int right_speed;
    switch (pinsB & 48) {
      case 16:
        left_speed = 5;
        break;
      case 32:
        left_speed = -5;
        break;
      default:
        left_speed = 0;
        break;
    }
    switch (pinsB & 192) {
      case 64:
        right_speed = 5;
        break;
      case 128:
        right_speed = -5;
        break;
      default:
        right_speed = 0;
        break;
    }
    if (simulation_state[2] != 'A') {  // STOP or PAUSE state
      left_speed = 0;
      right_speed = 0;
    }
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
    wb_led_set(left_led, (pinsB & 2) ? 1 : 0);
    wb_led_set(right_led, (pinsB & 8) ? 1 : 0);
    wb_robot_step(simulation_delay);

    pinsC = 2 * (int)wb_touch_sensor_get_value(left_bumper) + 8 * (int)wb_touch_sensor_get_value(right_bumper) +
            64 * (int)wb_touch_sensor_get_value(push_button);
    adcC2 = wb_distance_sensor_get_value(floor_sensor);
    if (adcC2 > 255)
      adcC2 = 255;
    if (previous_pinsC != pinsC || previous_adcC2 != adcC2) {
      previous_pinsC = pinsC;
      previous_adcC2 = adcC2;
      update_required = 1;
    } else
      update_required = 0;
  }
  if (pipe_handle)
    CloseHandle(pipe_handle);
  return 0;
}

#endif  // _WIN32
