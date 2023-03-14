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

//---------------------------------------------------------------------------------
//  Description:  Example C controller program for Nao robot.
//                This demonstrates how to script the behavior of several Nao robots
//---------------------------------------------------------------------------------

#include <assert.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/utils/motion.h>

#ifdef _MSC_VER
#define snprintf sprintf_s
#endif

static int time_step = -1;

static WbDeviceTag CameraTop, CameraBottom;
static WbDeviceTag accelerometer;
static WbDeviceTag leds[7];
static WbDeviceTag RShoulderPitch, LShoulderPitch, HeadYaw, HeadPitch;

static WbMotionRef currently_playing = NULL;

struct Sequence {
  int time;
  char *action;
  struct Sequence *next;
};

struct Motion {
  char *name;
  WbMotionRef ref;
  struct Motion *next;
} *motion_list = NULL;

static void find_and_enable_devices() {
  // camera
  CameraTop = wb_robot_get_device("CameraTop");
  CameraBottom = wb_robot_get_device("CameraBottom");
  wb_camera_enable(CameraTop, time_step);
  wb_camera_enable(CameraBottom, time_step);

  // accelerometer
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  // There are 7 controlable LED groups in Webots
  leds[0] = wb_robot_get_device("ChestBoard/Led");
  leds[1] = wb_robot_get_device("RFoot/Led");
  leds[2] = wb_robot_get_device("LFoot/Led");
  leds[3] = wb_robot_get_device("Face/Led/Right");
  leds[4] = wb_robot_get_device("Face/Led/Left");
  leds[5] = wb_robot_get_device("Ears/Led/Right");
  leds[6] = wb_robot_get_device("Ears/Led/Left");

  // shoulder pitch motors
  RShoulderPitch = wb_robot_get_device("RShoulderPitch");
  LShoulderPitch = wb_robot_get_device("LShoulderPitch");

  HeadYaw = wb_robot_get_device("HeadYaw");
  HeadPitch = wb_robot_get_device("HeadPitch");
}

// load motion files
static void load_motion_list() {
  const char *motion_dir = "../../motions";
  DIR *d = opendir(motion_dir);
  if (d) {
    struct dirent *dir;
    struct Motion *current_motion = NULL;
    while ((dir = readdir(d)) != NULL) {
      const char *name = dir->d_name;
      if (name[0] != '.') {
        struct Motion *new_motion = (struct Motion *)malloc(sizeof(struct Motion));
        if (current_motion == NULL)
          motion_list = new_motion;
        else
          current_motion->next = new_motion;
        current_motion = new_motion;
        current_motion->name = (char *)malloc(strlen(name) + 1);
        strcpy(current_motion->name, name);
        for (int i = 0; current_motion->name[i]; i++)
          if (current_motion->name[i] == '.') {
            current_motion->name[i] = '\0';
            break;
          }
        char filename[256];
        snprintf(filename, 256, "%s/%s.motion", motion_dir, current_motion->name);
        current_motion->ref = wbu_motion_new(filename);
        current_motion->next = NULL;
      }
    }
    closedir(d);
  }
}

static void free_motion_list() {
  while (motion_list) {
    free(motion_list->name);
    struct Motion *next = motion_list->next;
    free(motion_list);
    motion_list = next;
  }
}

static WbMotionRef find_motion(const char *name) {
  struct Motion *motion = motion_list;
  while (motion) {
    if (strcmp(motion->name, name) == 0)
      return motion->ref;
    motion = motion->next;
  }
  fprintf(stderr, "Motion not found: %s\n", name);
  return NULL;
}

static void start_motion(const char *name) {
  WbMotionRef motion = find_motion(name);
  // interrupt current motion
  if (currently_playing)
    wbu_motion_stop(currently_playing);

  // start new motion
  wbu_motion_play(motion);
  currently_playing = motion;
}

static struct Sequence *read_sequence(char color, char number) {
  struct Sequence *sequence, *first_sequence = NULL;
  char filename[32];
  snprintf(filename, sizeof(filename), "%c%d.txt", color, number);
  FILE *file = fopen(filename, "r");
  char buffer[256];
  char action[256];
  while (fgets(buffer, sizeof(buffer), file)) {
    int time;
    sscanf(buffer, "%d: %255s\n", &time, action);
    struct Sequence *new_sequence = (struct Sequence *)malloc(sizeof(struct Sequence));
    if (first_sequence == NULL)
      first_sequence = new_sequence;
    else
      sequence->next = new_sequence;
    sequence = new_sequence;
    sequence->next = new_sequence;
    sequence = new_sequence;
    sequence->time = time;
    sequence->action = (char *)malloc(strlen(action) + 1);
    strcpy(sequence->action, action);
    sequence->next = NULL;
  }
  fclose(file);
  return first_sequence;
}

static void free_sequence(struct Sequence *sequence) {
  while (sequence) {
    free(sequence->action);
    struct Sequence *next = sequence->next;
    free(sequence);
    sequence = next;
  }
}

static void run_action(const char *action) {
  if (strncmp(action, "motion:", 7) == 0)
    start_motion(&action[7]);
  else if (strncmp(action, "eyes:", 5) == 0) {
    unsigned int left, right;
    sscanf(&action[5], "%x", &left);
    sscanf(&action[12], "%x", &right);
    wb_led_set(leds[3], left);
    wb_led_set(leds[4], right);
    wb_led_set(leds[0], 0);
    wb_led_set(leds[1], 0);
    wb_led_set(leds[2], 0);
    wb_led_set(leds[5], 0);
    wb_led_set(leds[6], 0);
  } else if (strncmp(action, "yaw:", 4) == 0) {
    double yaw;
    sscanf(&action[4], "%lf", &yaw);
    wb_motor_set_position(HeadYaw, yaw);
  } else if (strncmp(action, "pitch:", 6) == 0) {
    double pitch;
    sscanf(&action[6], "%lf", &pitch);
    wb_motor_set_position(HeadPitch, pitch);
  } else if (strncmp(action, "left_arm:", 9) == 0) {
    double left_arm;
    sscanf(&action[9], "%lf", &left_arm);
    wb_motor_set_position(LShoulderPitch, left_arm);
  } else
    fprintf(stderr, "Unknown action: %s\n", action);
}

static void run_sequence(struct Sequence *sequence) {
  int time = wb_robot_get_time() * 1000;  // milliseconds
  while (sequence) {
    if (sequence->time == time)
      run_action(sequence->action);
    sequence = sequence->next;
  }
}

// main function
int main() {
  // call this before any other call to a Webots function
  wb_robot_init();
  char number, color;
  // simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();

  find_and_enable_devices();
  load_motion_list();
  // read keyboard and execute user commands
  const char *name = wb_robot_get_name();
  color = name[4];
  if (color == 'B')  // blue robot
    number = name[9] - '0';
  else  // red robot
    number = name[8] - '0';
  struct Sequence *sequence = read_sequence(color, number);
  while (1) {
    const double *acc = wb_accelerometer_get_values(accelerometer);
    if ((currently_playing == NULL || wbu_motion_is_over(currently_playing)) && fabs(acc[0]) > fabs(acc[1]) &&
        fabs(acc[0]) > fabs(acc[2]) && acc[0] < -5)
      start_motion("StandUpFromFront");
    else
      run_sequence(sequence);
    if (wb_robot_step(time_step) == -1)
      break;
  }
  wb_robot_cleanup();
  free_sequence(sequence);
  free_motion_list();
  return 0;
}
