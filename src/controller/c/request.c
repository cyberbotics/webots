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

#include "request.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // strlen, memcpy
#include <webots/types.h>
#include "scheduler.h"

WbRequest *request_new_from_data(const void *data, int size) {
  WbRequest *r = malloc(sizeof(WbRequest));
  r->data = malloc(size);
  memcpy(r->data, data, size);
  r->size = size;
  r->pointer = 0;
  r->immediate = false;
  return r;
}

WbRequest *request_new_empty() {
  WbRequest *r = malloc(sizeof(WbRequest));
  r->data = malloc(REQUEST_REALLOC_SIZE);
  r->size = REQUEST_REALLOC_SIZE;
  r->pointer = sizeof(int);  // room reserved for the size of the request
  return r;
}

void request_delete(WbRequest *r) {
  free(r->data);
  free(r);
}

void request_realloc(WbRequest *r, int additional_size) {
  r->size += additional_size;
  r->data = realloc(r->data, r->size);
}

void request_write_char(WbRequest *r, char c) {
  if (r->pointer + sizeof(char) > r->size)
    request_realloc(r, REQUEST_REALLOC_SIZE);
  r->data[r->pointer++] = c;
}

void request_write_uchar(WbRequest *r, unsigned char c) {
  request_write_char(r, (char)c);
}

void request_write_int16(WbRequest *r, short int i) {
  if (r->pointer + sizeof(i) > r->size)
    request_realloc(r, REQUEST_REALLOC_SIZE);
  *((short int *)(r->data + r->pointer)) = i;
  r->pointer += sizeof(i);
}

void request_write_uint16(WbRequest *r, unsigned short int i) {
  if (r->pointer + sizeof(i) > r->size)
    request_realloc(r, REQUEST_REALLOC_SIZE);
  *((unsigned short int *)(r->data + r->pointer)) = i;
  r->pointer += sizeof(i);
}

void request_write_int32(WbRequest *r, int i) {
  if (r->pointer + sizeof(i) > r->size)
    request_realloc(r, REQUEST_REALLOC_SIZE);
  *((int *)(r->data + r->pointer)) = i;
  r->pointer += sizeof(i);
}

void request_write_uint32(WbRequest *r, unsigned int i) {
  if (r->pointer + sizeof(i) > r->size)
    request_realloc(r, REQUEST_REALLOC_SIZE);
  *((unsigned int *)(r->data + r->pointer)) = i;
  r->pointer += sizeof(i);
}

void request_write_uint64(WbRequest *r, unsigned long long i) {
  if (r->pointer + sizeof(i) > r->size)
    request_realloc(r, REQUEST_REALLOC_SIZE);
  *((unsigned long long *)(r->data + r->pointer)) = i;
  r->pointer += sizeof(i);
}

void request_write_float(WbRequest *r, float f) {
  if (r->pointer + sizeof(f) > r->size)
    request_realloc(r, REQUEST_REALLOC_SIZE);
  *((float *)(void *)(r->data + r->pointer)) = f;
  r->pointer += sizeof(f);
}

void request_write_double(WbRequest *r, double d) {
  if (r->pointer + sizeof(double) > r->size)
    request_realloc(r, REQUEST_REALLOC_SIZE);
  *((double *)(void *)(r->data + r->pointer)) = d;
  r->pointer += sizeof(d);
}

void request_write_data(WbRequest *r, const void *data, int size) {
  if (r->pointer + size > r->size)
    request_realloc(r, size + REQUEST_REALLOC_SIZE);
  memcpy(r->data + r->pointer, data, size);
  r->pointer += size;
}

void request_write_string(WbRequest *r, const char *string) {
  request_write_data(r, string, strlen(string) + 1);
}

void request_write_size(WbRequest *r) {
  int size = r->pointer;
  memcpy(r->data, &size, sizeof(size));
}

short int request_read_int16(WbRequest *r) {
  short int i = *((short int *)(r->data + r->pointer));
  r->pointer += sizeof(i);
  return i;
}

unsigned short int request_read_uint16(WbRequest *r) {
  unsigned short int i = *((unsigned short int *)(r->data + r->pointer));
  r->pointer += sizeof(i);
  return i;
}

int request_read_int32(WbRequest *r) {
  int i = *((int *)(r->data + r->pointer));
  r->pointer += sizeof(i);
  return i;
}

unsigned int request_read_uint32(WbRequest *r) {
  unsigned int i = *((unsigned int *)(r->data + r->pointer));
  r->pointer += sizeof(i);
  return i;
}

float request_read_float(WbRequest *r) {
  float f = *((float *)(void *)(r->data + r->pointer));
  r->pointer += sizeof(f);
  return f;
}

double request_read_double(WbRequest *r) {
  double d = *((double *)(void *)(r->data + r->pointer));
  r->pointer += sizeof(d);
  return d;
}

void *request_read_data(WbRequest *r, int size) {
  void *p = r->data + r->pointer;
  r->pointer += size;
  return p;
}

char *request_read_string(WbRequest *r) {
  const char *src = r->data + r->pointer;
  const int l = strlen(src) + 1;
  char *dst = malloc(l);
  memcpy(dst, src, l);
  r->pointer += l;
  return dst;
}

bool request_is_over(WbRequest *r) {
  assert(r->pointer <= request_get_size(r));
  return r->pointer == request_get_size(r);
}

int request_get_size(WbRequest *r) {
  return *((int *)r->data);
}

int request_get_position(const WbRequest *r) {
  return r->pointer;
}

void request_set_position(WbRequest *r, int pos) {
  r->pointer = pos;
}

void request_set_immediate(WbRequest *r, bool immediate) {
  r->immediate = immediate;
}

bool request_is_immediate(const WbRequest *r) {
  return r->immediate;
}

void request_print(FILE *fd, WbRequest *r) {
  // store and reset position
  int position = request_get_position(r);
  request_set_position(r, 0);

  // print
  int size = request_get_size(r);
  fprintf(fd, "request %p\n", r);
  fprintf(fd, " size = %d\n", size);
  fprintf(fd, " time stamp= %d\n", *((int *)(r->data + sizeof(int))));
  fprintf(fd, " data = ");
  while (!request_is_over(r))
    fprintf(fd, "%#X;", request_read_uchar(r));
  fprintf(fd, "\n");
  fflush(fd);

  // restore position
  request_set_position(r, position);
}
