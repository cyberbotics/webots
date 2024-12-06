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

#include "abstract_camera.h"

#ifdef _WIN32
#include <windows.h>
#else  // memory mapped files
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

void wb_abstract_camera_cleanup(WbDevice *d) {
  AbstractCamera *c = d->pdata;
  if (c == NULL)
    return;
  image_cleanup(c->image);
  free(c->image);
  free(c);
}

void wb_abstract_camera_new(WbDevice *d, unsigned int id, int w, int h, double fov, double camnear, bool planar) {
  wb_abstract_camera_cleanup(d);
  AbstractCamera *c = malloc(sizeof(AbstractCamera));
  c->enable = false;
  c->unique_id = id;
  c->width = w;
  c->height = h;
  c->fov = fov;
  c->camnear = camnear;
  c->planar = planar;
  c->sampling_period = 0;
  c->image = image_new();

  d->pdata = c;
}

void wb_abstract_camera_write_request(WbDevice *d, WbRequest *r) {
  AbstractCamera *c = d->pdata;
  if (c->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, c->sampling_period);
    c->enable = false;  // done
  }
}

bool wb_abstract_camera_handle_command(WbDevice *d, WbRequest *r, unsigned char command) {
  bool command_handled = true;
  AbstractCamera *c = d->pdata;

  switch (command) {
    case C_CAMERA_MEMORY_MAPPED_FILE:
      // Cleanup the previous memory mapped file if any.
      image_cleanup(c->image);
      image_setup(c->image, r);
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

void wbr_abstract_camera_set_image(WbDevice *d, const unsigned char *image) {
  AbstractCamera *c = d->pdata;
  if (c && c->image->data)
    memcpy(c->image->data, image, 4 * c->height * c->width);
}

unsigned char *wbr_abstract_camera_get_image_buffer(WbDevice *d) {
  AbstractCamera *c = d->pdata;
  if (c && c->image->data)
    return c->image->data;
  return NULL;
}

void abstract_camera_allocate_image(WbDevice *d, int size) {
  AbstractCamera *c = d->pdata;
  if (c) {
    c->image->data = realloc(c->image->data, size);
    c->image->size = size;
  }
}

void wb_abstract_camera_enable(WbDevice *d, int sampling_period) {
  robot_mutex_lock();
  AbstractCamera *ac = d->pdata;

  if (ac) {
    ac->enable = true;
    ac->sampling_period = sampling_period;
  }
  robot_mutex_unlock();
}

int wb_abstract_camera_get_sampling_period(const WbDevice *d) {
  int sampling_period = 0;
  robot_mutex_lock();
  const AbstractCamera *ac = d->pdata;
  if (ac)
    sampling_period = ac->sampling_period;
  robot_mutex_unlock();
  return sampling_period;
}

int wb_abstract_camera_get_height(const WbDevice *d) {
  int result = -1;
  robot_mutex_lock();
  const AbstractCamera *ac = d->pdata;
  if (ac)
    result = ac->height;
  robot_mutex_unlock();
  return result;
}

int wb_abstract_camera_get_width(const WbDevice *d) {
  int result = -1;
  robot_mutex_lock();
  const AbstractCamera *ac = d->pdata;
  if (ac)
    result = ac->width;
  robot_mutex_unlock();
  return result;
}

double wb_abstract_camera_get_fov(const WbDevice *d) {
  double result = NAN;
  robot_mutex_lock();
  const AbstractCamera *ac = d->pdata;
  if (ac)
    result = ac->fov;
  robot_mutex_unlock();
  return result;
}

double wb_abstract_camera_get_near(const WbDevice *d) {
  double result = NAN;
  robot_mutex_lock();
  const AbstractCamera *ac = d->pdata;
  if (ac)
    result = ac->camnear;
  robot_mutex_unlock();
  return result;
}
