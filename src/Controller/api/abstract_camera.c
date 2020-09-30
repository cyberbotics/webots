/*
 * Copyright 1996-2020 Cyberbotics Ltd.
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

#include "abstract_camera.h"

#ifdef _WIN32
#include <windows.h>
#else  // POSIX shared memory segments
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/robot.h>
#include "messages.h"
#include "robot_private.h"

static void wb_abstract_camera_cleanup_shm(WbDevice *d) {
  AbstractCamera *c = d->pdata;
#ifdef _WIN32
  if (c->shm_file != NULL) {
    UnmapViewOfFile(c->image);
    CloseHandle(c->shm_file);
  }
#else  // POSIX shared memory
  if (c->shmid > 0) {
    munmap(c->image, c->shm_size);
    shm_unlink(c->shm_key);
  }
#endif
}

static void wb_abstract_camera_get_shm(WbDevice *d) {
  AbstractCamera *c = d->pdata;
#ifdef _WIN32
  c->shm_file = OpenFileMapping(FILE_MAP_WRITE, FALSE, c->shm_key);
  ROBOT_ASSERT(c->shm_file);
  c->image = MapViewOfFile(c->shm_file, FILE_MAP_WRITE, 0, 0, 0);
#else  // POSIX shared memory segments
  c->shmid = shm_open(c->shm_key, O_RDWR, 0400);
  c->image = (unsigned char *)mmap(0, c->shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, c->shmid, 0);
#endif

  ROBOT_ASSERT(c->image);
}

void wb_abstract_camera_cleanup(WbDevice *d) {
  AbstractCamera *c = d->pdata;
  if (c == NULL)
    return;
  wb_abstract_camera_cleanup_shm(d);
  free(c);
}

void wb_abstract_camera_new(WbDevice *d, unsigned int id, int w, int h, double fov, double camnear, bool spherical) {
  wb_abstract_camera_cleanup(d);
  AbstractCamera *c = malloc(sizeof(AbstractCamera));
  c->enable = false;
  c->unique_id = id;
  c->width = w;
  c->height = h;
  c->fov = fov;
  c->camnear = camnear;
  c->spherical = spherical;
  c->sampling_period = 0;
  c->shmid = -1;
  c->shm_size = 0;
  c->image = NULL;
  c->image_requested = false;
  c->image_update_time = 0.0;

  d->pdata = c;
}

void wb_abstract_camera_write_request(WbDevice *d, WbRequest *r) {
  AbstractCamera *c = d->pdata;
  if (c->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, c->sampling_period);
    c->enable = false;  // done
  }
  if (c->image_requested) {
    request_write_uchar(r, C_CAMERA_GET_IMAGE);
    c->image_requested = false;
  }
}

void wb_abstract_camera_update_timestamp(WbDevice *d) {
  AbstractCamera *c = d->pdata;
  c->image_update_time = wb_robot_get_time();
}

bool wb_abstract_camera_handle_command(WbDevice *d, WbRequest *r, unsigned char command) {
  bool command_handled = true;
  AbstractCamera *c = d->pdata;

  switch (command) {
    case C_CAMERA_SHARED_MEMORY:
      // Cleanup the previous shared memory if any.
      if (c->shm_size > 0)
        wb_abstract_camera_cleanup_shm(d);

      c->shm_size = request_read_int32(r);
      if (c->shm_size > 0) {
        c->shm_key = request_read_string(r);
        wb_abstract_camera_get_shm(d);
      }
      break;

    case C_CAMERA_GET_IMAGE:
      c->image_update_time = wb_robot_get_time();
      break;

    default:
      command_handled = false;
      break;
  }
  return command_handled;
}

void abstract_camera_toggle_remote(WbDevice *d, WbRequest *r) {
  AbstractCamera *c = d->pdata;
  if (c->sampling_period != 0)
    c->enable = true;
}

bool abstract_camera_request_image(AbstractCamera *ac, const char *functionName) {
  double current_simulation_time = wb_robot_get_time();
  const double previous_image_update_time = ac->image_update_time;  // in case of reset, time can go backward
  if (previous_image_update_time >= current_simulation_time)
    return true;

  ac->image_requested = true;
  wb_robot_flush_unlocked();
  if (!wb_robot_get_synchronization())
    // if controller is asynchronous, wb_robot_get_time() could be increased while requesting the image
    current_simulation_time = wb_robot_get_time();
  if (ac->image_update_time != current_simulation_time && previous_image_update_time <= ac->image_update_time &&
      robot_is_quitting() == 0) {
    fprintf(stderr, "Warning: %s: image could not be retrieved.\n", functionName);
    return false;
  }
  return true;
}

void wbr_abstract_camera_set_image(WbDevice *d, const unsigned char *image) {
  AbstractCamera *c = d->pdata;
  if (c && c->image)
    memcpy(c->image, image, 4 * c->height * c->width);
}

unsigned char *wbr_abstract_camera_get_image_buffer(WbDevice *d) {
  AbstractCamera *c = d->pdata;
  if (c && c->image)
    return c->image;
  return NULL;
}

void wb_abstract_camera_enable(WbDevice *d, int sampling_period) {
  robot_mutex_lock_step();
  AbstractCamera *ac = d->pdata;

  if (ac) {
    ac->enable = true;
    ac->sampling_period = sampling_period;
  }
  robot_mutex_unlock_step();
}

int wb_abstract_camera_get_sampling_period(WbDevice *d) {
  int sampling_period = 0;
  robot_mutex_lock_step();
  AbstractCamera *ac = d->pdata;
  if (ac)
    sampling_period = ac->sampling_period;
  robot_mutex_unlock_step();
  return sampling_period;
}

int wb_abstract_camera_get_height(WbDevice *d) {
  int result = -1;
  robot_mutex_lock_step();
  AbstractCamera *ac = d->pdata;
  if (ac)
    result = ac->height;
  robot_mutex_unlock_step();
  return result;
}

int wb_abstract_camera_get_width(WbDevice *d) {
  int result = -1;
  robot_mutex_lock_step();
  AbstractCamera *ac = d->pdata;
  if (ac)
    result = ac->width;
  robot_mutex_unlock_step();
  return result;
}

double wb_abstract_camera_get_fov(WbDevice *d) {
  double result = NAN;
  robot_mutex_lock_step();
  AbstractCamera *ac = d->pdata;
  if (ac)
    result = ac->fov;
  robot_mutex_unlock_step();
  return result;
}

double wb_abstract_camera_get_near(WbDevice *d) {
  double result = NAN;
  robot_mutex_lock_step();
  AbstractCamera *ac = d->pdata;
  if (ac)
    result = ac->camnear;
  robot_mutex_unlock_step();
  return result;
}
