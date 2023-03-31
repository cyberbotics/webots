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
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include <stdio.h>
#include <unistd.h>
#include <windows.h>

#define NAMED_PIPE_NAME "\\\\.\\Pipe\\picaxe-webots"

#define SIMULATION_STATE_START 1
#define SIMULATION_STATE_STOP 2
#define SIMULATION_STATE_PAUSE 3

static int pinsB = 0;
static int pinsC = 0;
static int dirsB = 0;
static int number_of_updates = 0;
static int simulation_delay = 0;
static double time_step = 0;
static int simulation_state = SIMULATION_STATE_STOP;
static int current_sound = -1;
static int suspended_sound = -1;

static WbDeviceTag gear_sensor = 0;
static WbDeviceTag gate = 0;
static WbDeviceTag gate_sensor = 0;
static WbDeviceTag rail = 0;
static WbDeviceTag closed_sensor = 0;
static WbDeviceTag open_sensor = 0;
static WbDeviceTag lamp = 0;
static WbDeviceTag optical_barrier = 0;
static WbDeviceTag inside_button = 0;
static WbDeviceTag outside_button = 0;

static char handle_message_line(const char *line) {
  if (strncmp(line, "number_of_updates:", 18) == 0) {
    sscanf(&line[18], "%d", &number_of_updates);
    return 0;
  }
  char return_value = 0;
  if (strncmp(line, "pinsB:", 6) == 0)
    sscanf(&line[6], "%d", &pinsB);
  else if (strncmp(line, "dirsB:", 6) == 0)
    sscanf(&line[6], "%d", &dirsB);
  else if (strncasecmp(line, "SimulationState:", 16) == 0) {
    // printf("simulation state: %s\n",&line[16]);
    if (strncasecmp(&line[16], "START", 5) == 0 || strncasecmp(&line[16], "STEP", 4) == 0)
      simulation_state = SIMULATION_STATE_START;
    else if (strncasecmp(&line[16], "STOP", 4) == 0)
      simulation_state = SIMULATION_STATE_STOP;
    else if (strncasecmp(&line[16], "PAUSE", 5) == 0)
      simulation_state = SIMULATION_STATE_PAUSE;
  } else if (strncasecmp(line, "SimulationDelay:", 16) == 0) {
    sscanf(&line[16], "%d", &simulation_delay);
    return_value = 1;
  } else if (strncasecmp(line, "SimulationType:", 15) == 0) {
    if (strncasecmp(&line[15], "PICAXE-28X2", 11) != 0) {
      fprintf(stderr, "ERROR: unsupported PICAXE type for portal: %s\n", &line[15]);
      fprintf(stderr, "Please select 'PICAXE-28X2' in PICAXE Editor ");
      fprintf(stderr, "from the 'Settings' tab in the Workspace Explorer.\n");
    }
  } else if (strncasecmp(line, "SimulationPinSize:", 18) == 0) {
    if (strncasecmp(&line[18], "28", 2) != 0)
      fprintf(stderr, "ERROR: unsupported pin size for portal: %s (should be 28)\n", &line[18]);
  }
  number_of_updates--;
  return return_value;
}

// returns 1 if simulation_delay was set
static char handle_message(const char *input_buffer) {
  char return_value = 0;
  int i = 0, l = strlen(input_buffer);
  if (l == 2)  // sometimes PE send a "\r\n" sequence which we can ignore
    return 0;
  do {
    handle_message_line(input_buffer);  // set number_of_updates
    // printf("number of updates=%d\n",number_of_updates);
    while (i < l && number_of_updates > 0) {
      if (input_buffer[i] == '\n') {
        if (handle_message_line(&input_buffer[i + 1]))
          return_value = 1;
        // printf("number of updates=%d\n",number_of_updates);
      }
      i++;
    }
    // printf("pinsB:%d\n",pinsB);
  } while (i < l);
  return return_value;
}

static const char *compute_update() {
  static char output_buffer[1024];
  snprintf(output_buffer, 1024, "number_of_updates:1\r\npinsC:%d\r\n", pinsC);
  return output_buffer;
}

static void play_sound(int i) {
  static const char *sound_file[2] = {"electric_motor_load.wav", "electric_motor_no_load.wav"};
  if (i == current_sound)
    return;
  int size = sizeof(sound_file) / sizeof(char *);
  if (i >= size)
    return;
  if (i == -1)
    PlaySound(NULL, 0, 0);
  else
    PlaySound(TEXT(sound_file[i]), NULL, SND_FILENAME | SND_ASYNC | SND_LOOP);
  current_sound = i;
}

int main() {
  HANDLE pipe_handle = NULL;
  char input_buffer[1024];
  char received_initialization = 0;
  wb_robot_init();
  gear_sensor = wb_robot_get_device("gear_sensor");
  gate = wb_robot_get_device("portal_motor::gate");
  gate_sensor = wb_robot_get_device("gate_sensor");
  rail = wb_robot_get_device("rail");
  closed_sensor = wb_robot_get_device("closed");
  open_sensor = wb_robot_get_device("open");
  outside_button = wb_robot_get_device("outside button");
  inside_button = wb_robot_get_device("inside button");
  optical_barrier = wb_robot_get_device("optical barrier");
  lamp = wb_robot_get_device("lamp");
  time_step = wb_robot_get_basic_time_step();
  wb_touch_sensor_enable(closed_sensor, time_step);
  wb_touch_sensor_enable(open_sensor, time_step);
  wb_touch_sensor_enable(outside_button, time_step);
  wb_touch_sensor_enable(inside_button, time_step);
  wb_position_sensor_enable(gear_sensor, time_step);
  wb_position_sensor_enable(gate_sensor, time_step);
  wb_keyboard_enable(time_step);
  printf("Keyboad controls:\n 'O': open portal\n 'C': close portal\n 'L': turn on light\n");
  for (;;) {
    if (pipe_handle == NULL) {
      pipe_handle =
        CreateFile(NAMED_PIPE_NAME, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
      if (pipe_handle == INVALID_HANDLE_VALUE)
        pipe_handle = NULL;  // try again later
      else
        printf("Connected with Picaxe software\n");
      received_initialization = 0;
    }
    if (pipe_handle) {
      DWORD e = NO_ERROR, nb_read = 0;
      if (!PeekNamedPipe(pipe_handle, NULL, 0, NULL, &nb_read, NULL)) {
        e = GetLastError();
        if (e == ERROR_BROKEN_PIPE) {
          fprintf(stderr, "Lost connection with PICAXE software\n");
          CloseHandle(pipe_handle);
          pipe_handle = NULL;
          nb_read = 0;
        } else
          fprintf(stderr, "PeekNamedPipe error = %d\n", (int)e);
      }
      if (nb_read) {
        DWORD total_read = 0;
        BOOL success = false;
        input_buffer[0] = 0;  // clear input buffer
        do {
          nb_read = 0;
          // printf("receiving data from PICAXE software\n");
          success = ReadFile(pipe_handle, &input_buffer[total_read], sizeof(input_buffer) - total_read, &nb_read, NULL);
          total_read += nb_read;
          input_buffer[total_read] = '\0';
          // printf("received %d bytes: %s\n",strlen(input_buffer),input_buffer);
          if (!success)
            e = GetLastError();
        } while (!success && e == ERROR_MORE_DATA);
        if (success) {
          input_buffer[total_read] = '\0';
          if (handle_message(input_buffer))
            received_initialization = 1;
        } else {
          if (e == ERROR_BROKEN_PIPE)
            fprintf(stderr, "Lost connection with PICAXE software\n");
          CloseHandle(pipe_handle);
          pipe_handle = NULL;
        }
      }
    }
    if (pipe_handle && received_initialization) {
      DWORD n;
      const char *update = compute_update();
      if (update) {
        // printf("sending:\n%s\n",update);
        if (WriteFile(pipe_handle, update, strlen(update) * sizeof(char) + 1, &n, NULL) == 0) {
          if (GetLastError() == ERROR_BROKEN_PIPE)
            fprintf(stderr, "Lost connection with PICAXE software\n");
          else
            fprintf(stderr, "Unknown error\n");
          CloseHandle(pipe_handle);
          pipe_handle = NULL;
        }
        fflush(stdout);
      }
    }
    const char k = wb_keyboard_get_key();
    if (k != -1)
      simulation_state = SIMULATION_STATE_START;
    if (simulation_state == SIMULATION_STATE_PAUSE) {
      if (current_sound != -1) {
        suspended_sound = current_sound;
        play_sound(-1);
      }
      usleep(time_step * 1000);
      continue;
    } else if (simulation_state == SIMULATION_STATE_START) {
      if (suspended_sound != -1) {
        play_sound(suspended_sound);
        suspended_sound = -1;
      }
    } else if (simulation_state == SIMULATION_STATE_STOP) {
      // reset every actuator to initial value
      double gate_position = wb_position_sensor_get_value(gate_sensor);
      if (gate_position > 0.0301 || gate_position < 0.0299)
        play_sound(0);
      else
        play_sound(-1);

      wb_motor_set_position(rail, 0);
      wb_motor_set_position(gate, 0.03);
      wb_led_set(lamp, 0);
      wb_robot_step(time_step);
      continue;
    }
    if (k == 'L')
      wb_led_set(lamp, 1);
    else
      wb_led_set(lamp, pinsB & dirsB & 1);
    double gate_position = wb_position_sensor_get_value(gate_sensor);
    if ((pinsB & dirsB & (64 + 128)) == 64 || k == 'C') {
      wb_motor_set_position(gate, 0);  // portal closed position
      if (gate_position <= 0.01) {     // derail position
        play_sound(1);
        wb_motor_set_position(rail, -0.005);
      } else {
        play_sound(0);
        wb_motor_set_position(rail, 0);
      }
    } else if ((pinsB & dirsB & (64 + 128)) == 128 || k == 'O') {
      wb_motor_set_position(gate, 0.3);  // portal open position
      if (gate_position >= 0.29) {       // derail position
        play_sound(1);
        wb_motor_set_position(rail, -0.005);
      } else {
        play_sound(0);
        wb_motor_set_position(rail, 0);
      }
    } else if ((pinsB & dirsB & (64 + 128)) == (64 + 128) || (pinsB & dirsB & (64 + 128)) == 0) {  // STOP the motors
      play_sound(-1);
      if (!isnan(gate_position))
        wb_motor_set_position(gate, gate_position);
    }
    if (pinsB & dirsB & 2)
      wb_distance_sensor_enable(optical_barrier, time_step);
    else
      wb_distance_sensor_disable(optical_barrier);
    wb_robot_step(time_step);
    pinsC = 1 * (int)wb_touch_sensor_get_value(outside_button) + 2 * (int)wb_touch_sensor_get_value(open_sensor) +
            4 * (int)wb_touch_sensor_get_value(closed_sensor) + 8 * (int)wb_touch_sensor_get_value(inside_button);
    if (pinsB & dirsB & 2) {
      pinsC += 16 * ((wb_distance_sensor_get_value(optical_barrier) < 936) ? 1 : 0);
    }
  }
  if (pipe_handle)
    CloseHandle(pipe_handle);
  return 0;
}

#endif  // _WIN32
