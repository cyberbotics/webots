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
  mtn.c: MTN Playback Support
  ---------------------------
  The semantics for MTN playback allow for *simultaneous* playback
  of several MTNs as long as they use separate joint sets.

  Thus, mtn_play(MTN*) would add an MTN to play *immediately* (not
  sequentially after the MTNs previously played are finished), in
  parallel with other MTNs currently running, as long as the joint
  set does not clash with joints currently being commanded by the
  MTNs which are already running.

  Also, mtn_is_over(MTN*) will decide whether given MTN is no longer
  playing, i.e. it is not in the list of current MTNs.
*/

#include "mtn.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/motor.h>
#include <webots/robot.h>

typedef struct _MTN_data MTN_data;

struct _MTN_data {
  int roll;
  int pitch;
  int yaw;
  int number_of_interpolation_frames;  // between previous frame and this frame
  int length;
  int *joint;  // x number_of_joints
};

struct _MTN {
  FILE *fd;
  short int major_version;
  short int minor_version;
  short int number_of_keyframes;  // number of MTN data structs
  short int frame_rate;
  short int current_keyframe;
  short int current_keyframe_time;
  int current_time;
  int length;
  char *motion_name;
  char *creator;
  char *design_label;
  short int number_of_joints;
  char **prm;
  int *device_tag;
  MTN_data *data;  // x number_of_keyframes
  MTN *next;       // next MTN in list
};

static MTN *mtn_list = NULL;
static char mtn_last_error[512] = "";

static void list_append(MTN *mtn) {
  if (mtn_list) {
    MTN *list = mtn_list;
    while (list->next)
      list = list->next;
    list->next = mtn;
  } else
    mtn_list = mtn;
}

static void list_remove(const MTN *mtn) {
  MTN *prev = NULL;
  MTN *tmp = mtn_list;

  while (tmp) {
    if (tmp == mtn) {
      if (prev)
        prev->next = tmp->next;

      if (mtn_list == tmp)
        mtn_list = mtn_list->next;

      tmp->next = NULL;
      return;
    }

    prev = tmp;
    tmp = tmp->next;
  }
}

static int list_contains(const MTN *mtn) {
  MTN *list = mtn_list;

  while (list) {
    if (list == mtn)
      return 1;
    list = list->next;
  }

  return 0;
}

static void _fread(void *ptr, size_t size, size_t nitems, FILE *stream) {
  // cppcheck-suppress uninitvar
  size_t ret = fread(ptr, size, nitems, stream);
  if (ret != nitems)
    fprintf(stderr, "Error during _fread\n");
  return;
}

static char *mtn_read_string(FILE *fd) {
  unsigned char length;
  char *buffer;
  _fread(&length, 1, 1, fd);
  buffer = (char *)malloc(length + 1);
  _fread(buffer, 1, length, fd);
  buffer[length] = '\0';
  return buffer;
}

static void mtn_set_error(const char *format, ...) {
  va_list args;
  va_start(args, format);
  vsprintf(mtn_last_error, format, args);
  va_end(args);
}

static int mtn_find_devices(MTN *mtn) {
  int i;
  for (i = 0; i < mtn->number_of_joints; i++) {
    WbDeviceTag tag = wb_robot_get_device(mtn->prm[i]);
    if (tag == 0) {  // not found
      mtn_set_error("Missing device in model: %s\n", mtn->prm[i]);
      return 0;
    }
    mtn->device_tag[i] = tag;
  }
  return 1;  // everything went fine
}

/*
<sp/13.09.2004>  NOTE:

 Problems
 --------
 While this implementation "sort of" works, there are two major problems:
 (a) we do not check that the robot is in initial position at start of MTN
     and at final position at end of MTN; this is a problem because the
     playback might become highly out-of-synch if the initial keyframe is far
     from the current positions, and also because mtn_is_over() might return
     'true' while the last keyframe has not yet been reached;
 (b) the movement generated will be jumpy if the number of interpolation
     frames between keyframes is high (this is because joint positioning orders
     are issued at keyframe start, and there is no notion of speed other than
     that the next keyframe will not start until a time corresponding to the
     interpolation timespan has passed).

 Solutions
 ---------
 There is currently no workaround for this problem, except to generate
 0-keyframe-interpolation MTNs. In order to correct the problems, we need to:
 (a) implement mtn_reach_frame(MTN*,int*frame), where 'frame' is the desired
     joints position data, which would use set_motor_position with max velocity
     and return 'true' when that position is reached, and use it at start and
     end of MTN;
 (b) manually generate interpolation keyframes, using either linear or spline
     methods (this is what Motion Editor for Aibo ERS-7 does); this way,
     keyframes will all be equally sampled at 16ms and everybody will be happy
     (provided the MTN defines a valid movement as per joint velocity limits);
 (b)' alternatively, we might use motor_set_velocity() to some appropriate
    value prior to setting joint position; note that while this solution is
    more elegant, it would involve solving (a) first.
*/
void mtn_step(int ms) {
  // loop on running MTNs
  MTN *mtn;
  for (mtn = mtn_list; mtn; mtn = mtn->next) {
    // start of keyframe? => set joints command on keyframe start
    if (mtn->current_keyframe_time == 0) {
      /* printf("... %s @ %d[ms]: keyframe %d (i:%d): SET\n",
              mtn->motion_name,mtn->current_time,
              mtn->current_keyframe,
              mtn->data[mtn->current_keyframe].number_of_interpolation_frames);*/
      int j;
      for (j = 0; j < mtn->number_of_joints; j++)
        wb_motor_set_position(mtn->device_tag[j], mtn->data[mtn->current_keyframe].joint[j] / 1000000.0);
    }
    // updates for next call
    mtn->current_keyframe_time += ms;
    mtn->current_time += ms;

    // keyframe over? => pass on to next keyframe
    if (mtn->current_keyframe_time >= mtn->data[mtn->current_keyframe].length) {
      mtn->current_keyframe++;
      mtn->current_keyframe_time = 0;
      // last keyframe reached? => mtn over, remove from running list
      if (mtn->current_keyframe >= mtn->number_of_keyframes) {
        mtn->current_keyframe = 0;
        mtn->current_time = 0;
        list_remove(mtn);
      }
    }
  }
}

MTN *mtn_new(const char *filename) {
  int size, position;
  int i, j;

  MTN *mtn = (MTN *)malloc(sizeof(MTN));
  mtn->fd = fopen(filename, "rb");

  if (mtn->fd == 0) {
    mtn_set_error("File not found: %s\n", filename);
    free(mtn);
    return NULL;
  }

  char magic[5];
  _fread(magic, 1, 4, mtn->fd);
  magic[4] = '\0';

  if (strcmp(magic, "OMTN") != 0) {
    mtn_set_error("Wrong MTN magic: %s (should be OMTN)\n", magic);
    fclose(mtn->fd);
    free(mtn);
    return NULL;
  }

  // Section 0
  // ignore section number,section size and number of sections
  fseek(mtn->fd, 12, SEEK_CUR);
  _fread(&mtn->major_version, 2, 1, mtn->fd);
  _fread(&mtn->minor_version, 2, 1, mtn->fd);
  _fread(&mtn->number_of_keyframes, 2, 1, mtn->fd);
  _fread(&mtn->frame_rate, 2, 1, mtn->fd);
  fseek(mtn->fd, 4, SEEK_CUR);  // ignore reserved

  // Section 1
  position = ftell(mtn->fd);
  fseek(mtn->fd, 4, SEEK_CUR);  // ignore section number
  _fread(&size, 4, 1, mtn->fd);
  mtn->motion_name = mtn_read_string(mtn->fd);
  mtn->creator = mtn_read_string(mtn->fd);
  mtn->design_label = mtn_read_string(mtn->fd);
  position += size;
  fseek(mtn->fd, position, SEEK_SET);

  // Section 2
  fseek(mtn->fd, 4, SEEK_CUR);  // ignore section number
  _fread(&size, 4, 1, mtn->fd);
  _fread(&mtn->number_of_joints, 2, 1, mtn->fd);
  mtn->prm = (char **)malloc(sizeof(char *) * mtn->number_of_joints);
  mtn->device_tag = (int *)malloc(sizeof(int) * mtn->number_of_joints);

  for (i = 0; i < mtn->number_of_joints; i++) {
    mtn->prm[i] = mtn_read_string(mtn->fd);
    mtn->device_tag[i] = 0;
  }

  position += size;
  fseek(mtn->fd, position, SEEK_SET);

  // Section 3
  // ignore section number,section size and data type
  fseek(mtn->fd, 12, SEEK_CUR);
  mtn->data = (MTN_data *)malloc(sizeof(MTN_data) * mtn->number_of_keyframes);
  mtn->length = 0;  // sum of all keyframe lengths
  for (i = 0; i < mtn->number_of_keyframes; i++) {
    if (i > 0)
      _fread(&mtn->data[i].number_of_interpolation_frames, 4, 1, mtn->fd);
    else
      mtn->data[i].number_of_interpolation_frames = 0;
    _fread(&mtn->data[i].roll, 4, 1, mtn->fd);
    _fread(&mtn->data[i].pitch, 4, 1, mtn->fd);
    _fread(&mtn->data[i].yaw, 4, 1, mtn->fd);
    mtn->data[i].joint = (int *)malloc(sizeof(int) * mtn->number_of_joints);

    for (j = 0; j < mtn->number_of_joints; j++)
      _fread(&mtn->data[i].joint[j], 4, 1, mtn->fd);

    mtn->data[i].length = 0;
    // compute previous keyframe length & update mtn length
    if (i > 0) {
      mtn->data[i - 1].length = (mtn->data[i].number_of_interpolation_frames + 1) * mtn->frame_rate;
      mtn->length += mtn->data[i - 1].length;
    }
  }

  // printf("read %ld bytes\n",ftell(mtn->fd));
  fclose(mtn->fd);

  // finalize
  mtn->current_time = 0;
  mtn->current_keyframe_time = 0;
  mtn->current_keyframe = 0;
  mtn->next = NULL;

  // check that all referenced devices are valid
  if (!mtn_find_devices(mtn)) {
    mtn_delete(mtn);
    mtn = NULL;
  }
  return mtn;
}

void mtn_delete(MTN *mtn) {
  if (mtn == NULL)
    return;

  int i;
  for (i = 0; i < mtn->number_of_joints; i++)
    free(mtn->prm[i]);

  free(mtn->prm);
  free(mtn->device_tag);

  for (i = 0; i < mtn->number_of_keyframes; i++)
    free(mtn->data[i].joint);

  free(mtn->data);
  free(mtn->motion_name);
  free(mtn->creator);
  free(mtn->design_label);
  free(mtn);
}

int mtn_get_length(MTN *mtn) {
  return mtn->length;
}

int mtn_get_time(MTN *mtn) {
  return mtn->current_time;
}

void mtn_play(MTN *mtn) {
  list_append(mtn);
  // TODO: check for possible joint control clashes before  appending to the running list
}

int mtn_is_over(MTN *mtn) {
  return !list_contains(mtn);
  // rationale: an MTN is over when it reaches its final position;
  // when that happens, it is removed from the simultaneous play queue
}

const char *mtn_get_error() {
  return mtn_last_error;
}

void mtn_fprint(FILE *fd, MTN *mtn) {
  int i, j;
  fprintf(fd, "MTN version:\t%d.%d\n", mtn->major_version, mtn->minor_version);
  fprintf(fd, "# keyframes:\t%d\n", mtn->number_of_keyframes);
  fprintf(fd, "Frame rate:\t%d\n", mtn->frame_rate);
  fprintf(fd, "Motion name:\t%s\n", mtn->motion_name);
  fprintf(fd, "Creator:\t%s\n", mtn->creator);
  fprintf(fd, "Design label:\t%s\n", mtn->design_label);
  fprintf(fd, "# joints:\t%d\n", mtn->number_of_joints);

  for (i = 0; i < mtn->number_of_joints; i++)
    fprintf(fd, " Joints %d:\t%s %d\n", i, mtn->prm[i], mtn->device_tag[i]);

  for (i = 0; i < mtn->number_of_keyframes; i++) {
    fprintf(fd, "(%d,%d,%d) - %d:", mtn->data[i].roll, mtn->data[i].pitch, mtn->data[i].yaw,
            mtn->data[i].number_of_interpolation_frames);

    for (j = 0; j < mtn->number_of_joints; j++)
      fprintf(fd, " %d", mtn->data[i].joint[j]);

    fprintf(fd, "\n");
  }
  fprintf(fd, "Total length: %d ms\n", mtn->length);
}
