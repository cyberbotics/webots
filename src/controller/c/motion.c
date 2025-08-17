/*
 * Copyright 1996-2024 Cyberbotics Ltd.
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

//-----------------------------------------------------------
// Purpose:  Webots API for loading and playing .motion files
//-----------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/device.h>
#include <webots/types.h>
#include <webots/utils/motion.h>
#include "robot_private.h"

typedef struct WbMotionStructPrivate {
  int n_joints;        // number of joints (columns) in the file
  int n_poses;         // number of poses (lines) in the file
  char *filename;      // file name
  char **joint_names;  // array of n_joints joint names
  WbDeviceTag *tags;   // array of n_joints device tags
  WbNodeType *types;   // array of n_joints device types
  int *times;          // array of n_poses poses
  double **pos;        // two-dimensional [n_poses][n_joints] array
  int elapsed;         // elapsed time when playing this motion file
  bool playing;        // is currenly playing
  bool reverse;        // playing forward of backwards ?
  bool loop;           // loop when reaching the end (or begining) ?
  WbMotionRef next;    // next struct in list
} WbMotionStruct;

extern void wb_motor_set_position_no_mutex(WbDeviceTag, double);

static const int UNDEFINED_TIME = -1;
static const int MAX_LINE = 4096;
static const double UNDEFINED_POSITION = -9999999.9;
static WbMotionRef head = NULL;
static const char *HEADER = "#WEBOTS_MOTION";
static const char *VERSION = "V1.0";
static int cleanup_done = 0;

// string to time conversion
// acceptable input format: [[Minutes:]Seconds:]:Milliseconds
// returns UNDEFINED_TIME in case of syntax error
static int str_to_time(const char *token) {
  // check for illegal characters
  const char cset[] = "0123456789:";
  if (strspn(token, cset) < strlen(token))
    return UNDEFINED_TIME;

  // count number of colons
  int k = 0;
  const char *p = token;
  while (*p) {
    if (*p == ':')
      k++;
    p++;
  }

  int v[3] = {0, 0, 0};
  int r;
  switch (k) {
    case 0:
      r = sscanf(token, "%d", &v[2]);
      break;
    case 1:
      r = sscanf(token, "%d:%d", &v[1], &v[2]);
      break;
    case 2:
      r = sscanf(token, "%d:%d:%d", &v[0], &v[1], &v[2]);
      break;
    default:
      return UNDEFINED_TIME;
  }

  // unexpected number of items
  if (r != k + 1)
    return UNDEFINED_TIME;

  return v[0] * 60000 + v[1] * 1000 + v[2];
}

static char *next_token(char *buffer) {
  return strtok(buffer, ",\n\r");
}

// check the file syntax and find out the number of joints and poses
// returns true only if the syntax is 100% correct
static bool motion_check_file(FILE *file, const char *filename, int *n_joints, int *n_poses) {
  // line buffer and counter
  char buffer[MAX_LINE];
  int line = 1;
  int joints = 0;

  if (!file || fgets(buffer, MAX_LINE, file) == NULL) {
    fprintf(stderr, "Error: wbu_motion_new(): file '%s' is empty.\n", filename);
    return false;
  }

  const char *token = next_token(buffer);
  if (!token) {
    fprintf(stderr, "Error: wbu_motion_new(): unexpected end of file '%s'.\n", filename);
    return false;
  }

  if (strcmp(token, HEADER) != 0) {
    fprintf(stderr, "Error: wbu_motion_new(): invalid motion file header in file '%s'.\n", filename);
    return false;
  }

  token = next_token(NULL);
  if (!token) {
    fprintf(stderr, "Error: wbu_motion_new(): unexpected end of file '%s'.\n", filename);
    return false;
  }

  if (strcmp(token, VERSION) != 0) {
    fprintf(stderr, "Error: wbu_motion_new(): unsupported version number '%s' in motion file '%s'.\n", token, filename);
    return false;
  }

  // count joints (columns)
  token = next_token(NULL);
  while (token) {
    joints++;
    token = next_token(NULL);
  }

  int time = -1;

  // read line by line
  while (fgets(buffer, MAX_LINE, file) != NULL) {
    token = next_token(buffer);

    // empty line found
    if (!token)
      break;

    line++;

    // verify time value
    int temp = str_to_time(token);
    if (temp == UNDEFINED_TIME) {
      fprintf(stderr, "Error: wbu_motion_new(): expected <time value> but found '%s', in file '%s', line %d.\n", token,
              filename, line);
      return false;
    }

    // verify increasing time sequence
    if (temp <= time) {
      fprintf(stderr, "Error: wbu_motion_new(): illegal (non-increasing) time sequence detected in file '%s', line %d.\n",
              filename, line);
      return false;
    }

    time = temp;

    // skip pose name
    token = next_token(NULL);
    if (!token) {
      fprintf(stderr, "Error: wbu_motion_new(): pose name expected in file '%s', line %d.\n", filename, line);
      return false;
    }

    int i;
    for (i = 0; i < joints; i++) {
      token = next_token(NULL);
      if (!token) {
        fprintf(stderr, "Error: wbu_motion_new(): missing data in file '%s', line %d.\n", filename, line);
        return false;
      }

      // parse data element
      if (strcmp(token, "*") != 0) {
        double f;
        if (sscanf(token, "%lf", &f) != 1) {
          fprintf(stderr, "Error: wbu_motion_new(): expected <double value>, but found '%s', in file '%s', line %d.\n", token,
                  filename, line);
          return false;
        }
      }
    }

    // check for extra tokens
    token = next_token(NULL);
    if (token) {
      fprintf(stderr, "Error: wbu_motion_new(): unexpected extra data '%s', in file '%s', line %d.\n", token, filename, line);
      return false;
    }
  }

  *n_joints = joints;
  *n_poses = line - 1;

  return true;
}

// load .motion file
// warning: this function assumes a 100% correct file syntax!
// prerequisite: the file syntax must have been verified with motion_check_file() and n_joints and n_poses must have been set
// accordingly
static void motion_load(WbMotionRef ref, FILE *file) {
  // back to beginning of file
  rewind(file);

  char buffer[MAX_LINE];
  const char *r = fgets(buffer, MAX_LINE, file);
  if (r == NULL)
    return;  // should never happen

  // skip header and version number
  next_token(buffer);
  next_token(NULL);

  int i;
  for (i = 0; i < ref->n_joints; i++) {
    const char *token = next_token(NULL);
    ref->joint_names[i] = malloc(strlen(token) + 1);
    strcpy(ref->joint_names[i], token);
  }

  for (i = 0; i < ref->n_poses; i++) {
    r = fgets(buffer, MAX_LINE, file);
    if (r == NULL)
      return;  // should never happen
    const char *token = next_token(buffer);
    ref->times[i] = str_to_time(token);
    next_token(NULL);  // skip pose name
    int j;
    for (j = 0; j < ref->n_joints; j++) {
      token = next_token(NULL);
      if (strcmp(token, "*") == 0)
        ref->pos[i][j] = UNDEFINED_POSITION;
      else
        sscanf(token, "%lf", &ref->pos[i][j]);
    }
  }
}

static WbDeviceTag motion_find_device_tag(const char *joint_name) {
  int n_devices = robot_get_number_of_devices();

  int tag;
  for (tag = 0; tag < n_devices; tag++) {
    if (strcmp(robot_get_device_name(tag), joint_name) == 0)
      return tag;
  }

  return 0;
}

static double motion_compute_joint_pos(WbMotionRef ref, int joint) {
  ROBOT_ASSERT(ref && joint >= 0 && joint < ref->n_joints);
  // cppcheck-suppress nullPointerRedundantCheck
  if (ref->n_poses == 0)
    return UNDEFINED_POSITION;

  // shorter to write
  int elapsed = ref->elapsed;
  int last = ref->n_poses - 1;

  if (elapsed <= ref->times[0])
    return ref->pos[0][joint];

  if (elapsed >= ref->times[last])
    return ref->pos[last][joint];

  int a, b;
  for (a = 0, b = 1; b < ref->n_poses; a++, b++) {
    if (elapsed >= ref->times[a] && elapsed <= ref->times[b]) {
      if (ref->pos[a][joint] == UNDEFINED_POSITION || ref->pos[b][joint] == UNDEFINED_POSITION)
        return UNDEFINED_POSITION;
      else
        // linear interpolation between the two joint positions
        return ref->pos[a][joint] +
               (ref->elapsed - ref->times[a]) * (ref->pos[b][joint] - ref->pos[a][joint]) / (ref->times[b] - ref->times[a]);
    }
  }

  ROBOT_ASSERT(0);
  return UNDEFINED_POSITION;
}

static void motion_actuate(WbMotionRef ref) {
  ROBOT_ASSERT(ref);

  int j;
  // cppcheck-suppress nullPointerRedundantCheck
  for (j = 0; j < ref->n_joints; j++) {
    if (ref->tags[j]) {
      double pos = motion_compute_joint_pos(ref, j);
      if (pos != UNDEFINED_POSITION) {
        if (ref->types[j] == WB_NODE_ROTATIONAL_MOTOR || ref->types[j] == WB_NODE_LINEAR_MOTOR)
          wb_motor_set_position_no_mutex(ref->tags[j], pos);
        else
          fprintf(stderr, "Error: unexpeced type for device \"%s\".\n", ref->joint_names[j]);
      }
    }
  }
}

static inline int motion_get_duration(WbMotionRef ref) {
  return ref->times[ref->n_poses - 1];
}

static void motion_play_step(WbMotionRef ref, int millis) {
  // set motor target postions
  motion_actuate(ref);

  int duration = motion_get_duration(ref);

  if (ref->reverse) {
    if (ref->elapsed <= 0) {
      if (ref->loop)
        ref->elapsed = duration;
      else {
        ref->elapsed = 0;
        ref->playing = false;
      }
    } else
      ref->elapsed -= millis;
  } else {
    if (ref->elapsed >= duration) {
      if (ref->loop)
        ref->elapsed = 0;
      else {
        ref->elapsed = duration;
        ref->playing = false;
      }
    } else
      ref->elapsed += millis;
  }
}

static void motion_enqueue(WbMotionRef ref) {
  ref->next = head;
  head = ref;
}

static bool motion_dequeue(WbMotionRef ref) {
  WbMotionRef *p = &head;
  while (*p) {
    if (*p == ref) {
      *p = ref->next;
      return true;
    }
    p = &(*p)->next;
  }

  return false;
}

// functions called from robot.c below

void motion_step_all(int millis) {
  if (millis <= 0)
    return;

  WbMotionRef ref = head;
  while (ref) {
    if (ref->playing)
      motion_play_step(ref, millis);

    ref = ref->next;
  }
}

void motion_cleanup() {
  while (head)
    wbu_motion_delete(head);

  ROBOT_ASSERT(head == NULL);
  cleanup_done = 1;
}

// Webots API functions below

WbMotionRef wbu_motion_new(const char *filename) {
  if (!filename || !filename[0]) {
    fprintf(stderr, "Error: %s() called with NULL or empty 'filename' argument.\n", __FUNCTION__);
    return NULL;
  }

  FILE *file = fopen(filename, "r");
  if (!file) {
    fprintf(stderr, "Error: %s(): could not open '%s' file.\n", __FUNCTION__, filename);
    return NULL;
  }

  // check file headers and syntax
  int n_joints = 0, n_poses = 0;
  if (!motion_check_file(file, filename, &n_joints, &n_poses))
    return NULL;

  if (n_joints == 0 || n_poses == 0) {
    fprintf(stderr, "Error: %s(): file '%s' must contain at least one joint and one pose.\n", __FUNCTION__, filename);
    return NULL;
  }

  // alloc and initialize all fields
  WbMotionRef ref = malloc(sizeof(WbMotionStruct));
  ref->elapsed = 0;
  ref->playing = false;
  ref->reverse = false;
  ref->loop = false;
  ref->n_joints = n_joints;
  ref->n_poses = n_poses;
  ref->filename = malloc(strlen(filename) + 1);
  strcpy(ref->filename, filename);
  ref->joint_names = malloc(n_joints * sizeof(char *));
  int i;
  for (i = 0; i < n_joints; i++)
    ref->joint_names[i] = NULL;
  ref->times = malloc(n_poses * sizeof(int));
  ref->pos = malloc(n_poses * sizeof(double *));
  for (i = 0; i < n_poses; i++)
    ref->pos[i] = malloc(n_joints * sizeof(double));

  // syntax is OK: load now
  motion_load(ref, file);
  fclose(file);

  // associate device tags to joint names found in file
  ref->tags = malloc(n_joints * sizeof(WbDeviceTag));
  ref->types = malloc(n_joints * sizeof(WbNodeType));
  for (i = 0; i < n_joints; i++) {
    ref->tags[i] = motion_find_device_tag(ref->joint_names[i]);
    ref->types[i] = wb_device_get_node_type(ref->tags[i]);
    if (!ref->tags[i])
      fprintf(stderr, "Warning: %s(): ignoring joint '%s', specified in file '%s', but not found in this robot.\n",
              __FUNCTION__, ref->joint_names[i], filename);
  }

  // enqueue self
  motion_enqueue(ref);

  return ref;
}

void wbu_motion_delete(WbMotionRef motion) {
  if (!motion)
    return;
  // dequeue self
  if (!motion_dequeue(motion)) {
    if (cleanup_done) {            // the Python API calls first motion_cleanup and then tries to delete motion files.
      ROBOT_ASSERT(head == NULL);  // no new motion should have been created after the cleanup.
    } else
      fprintf(stderr, "Error: %s(): attempt to delete an invalid 'motion'.\n", __FUNCTION__);
    return;
  }

  free(motion->filename);
  int i;
  for (i = 0; i < motion->n_joints; i++)
    free(motion->joint_names[i]);
  free(motion->joint_names);
  free(motion->tags);
  free(motion->types);
  free(motion->times);
  for (i = 0; i < motion->n_poses; i++)
    free(motion->pos[i]);
  free(motion->pos);
  free(motion);
}

void wbu_motion_play(WbMotionRef motion) {
  if (!motion) {
    fprintf(stderr, "Error: %s() called with NULL 'motion' argument.\n", __FUNCTION__);
    return;
  }

  motion->playing = true;

  // if we reached either end: restart from other end
  int duration = motion_get_duration(motion);
  if (motion->reverse && motion->elapsed <= 0)
    motion->elapsed = duration;
  else if (!motion->reverse && motion->elapsed >= duration)
    motion->elapsed = 0;
}

int wbu_motion_get_duration(WbMotionRef motion) {
  if (!motion) {
    fprintf(stderr, "Error: %s() called with NULL argument.\n", __FUNCTION__);
    return -1;
  }

  return motion_get_duration(motion);
}

int wbu_motion_get_time(WbMotionRef motion) {
  if (!motion) {
    fprintf(stderr, "Error: %s() called with NULL argument.\n", __FUNCTION__);
    return -1;
  }

  return motion->elapsed;
}

void wbu_motion_set_time(WbMotionRef motion, int time) {
  if (!motion) {
    fprintf(stderr, "Error: %s() called with NULL argument.\n", __FUNCTION__);
    return;
  }

  if (time < 0)
    motion->elapsed = 0;
  else if (time > motion_get_duration(motion))
    motion->elapsed = motion_get_duration(motion);
  else
    motion->elapsed = time;
}

bool wbu_motion_is_over(WbMotionRef motion) {
  if (!motion) {
    fprintf(stderr, "Error: %s() called with NULL argument.\n", __FUNCTION__);
    return false;
  }

  if (motion->loop)
    return false;
  else
    return (motion->reverse && motion->elapsed <= 0) || (!motion->reverse && motion->elapsed >= motion_get_duration(motion));
}

void wbu_motion_stop(WbMotionRef motion) {
  if (!motion) {
    fprintf(stderr, "Error: %s() called with NULL argument.\n", __FUNCTION__);
    return;
  }

  motion->playing = false;
}

void wbu_motion_set_reverse(WbMotionRef motion, bool reverse) {
  if (!motion) {
    fprintf(stderr, "Error: %s() called with NULL argument.\n", __FUNCTION__);
    return;
  }

  motion->reverse = reverse;
}

void wbu_motion_set_loop(WbMotionRef motion, bool loop) {
  if (!motion) {
    fprintf(stderr, "Error: %s() called with NULL argument.\n", __FUNCTION__);
    return;
  }

  motion->loop = loop;
}
