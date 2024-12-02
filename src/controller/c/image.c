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

#include "image_private.h"

#include <webots/robot.h>
#include "robot_private.h"

#ifdef _WIN32
#include <windows.h>
#else  // memory mapped files
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
#include <stdlib.h>

Image *image_new() {
  Image *i = malloc(sizeof(Image));
  i->filename = NULL;
  i->size = 0;
  i->data = NULL;
  return i;
}

void image_cleanup(Image *i) {
  if (i->size <= 0)
    return;

#ifdef _WIN32
  if (i->fd != NULL) {
    UnmapViewOfFile(i->data);
    CloseHandle(i->fd);
  }
#else
  if (i->data > 0)
    munmap(i->data, i->size);
#endif
}

void image_setup(Image *i, WbRequest *r) {
  i->size = request_read_int32(r);
  if (i->size > 0) {
    i->filename = request_read_string(r);
    image_get(i);
  }
}

void image_get(Image *i) {
#ifdef _WIN32
  i->fd = OpenFileMapping(FILE_MAP_WRITE, FALSE, i->filename);
  ROBOT_ASSERT(i->fd);
  i->data = MapViewOfFile(i->fd, FILE_MAP_WRITE, 0, 0, 0);
#else  // memory mapped files
  int fd = open(i->filename, O_RDWR, 0400);
  i->data = (unsigned char *)mmap(0, i->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
#endif
  ROBOT_ASSERT(i->data);
}
