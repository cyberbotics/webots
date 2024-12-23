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

#ifndef REQUEST_H
#define REQUEST_H

#include <stdio.h>
#include <webots/types.h>

#define REQUEST_REALLOC_SIZE 1024
#define request_write_tag request_write_uint16
#define request_read_tag request_read_uint16

struct _WbRequest {
  int pointer;
  int size;  // size of the memory chunk
  char *data;
  bool immediate;
};

typedef struct _WbRequest WbRequest;

WbRequest *request_new_from_data(const void *data, int size);
WbRequest *request_new_empty();
void request_delete(WbRequest *);
void request_reset(WbRequest *);
void request_realloc(WbRequest *, int additional_size);
void request_write_char(WbRequest *, char);
void request_write_uchar(WbRequest *, unsigned char);
void request_write_int16(WbRequest *, short);
void request_write_uint16(WbRequest *, unsigned short);
void request_write_int32(WbRequest *, int);
void request_write_uint32(WbRequest *, unsigned int);
void request_write_uint64(WbRequest *, unsigned long long);
void request_write_float(WbRequest *, float);
void request_write_double(WbRequest *, double);
void request_write_data(WbRequest *, const void *data, int size);
void request_write_string(WbRequest *, const char *);
void request_write_size(WbRequest *);
char request_read_char(WbRequest *);
unsigned char request_read_uchar(WbRequest *);
short request_read_int16(WbRequest *);
unsigned short request_read_uint16(WbRequest *);
int request_read_int32(WbRequest *);
unsigned int request_read_uint32(WbRequest *);
float request_read_float(WbRequest *);
double request_read_double(WbRequest *);
void *request_read_data(WbRequest *, int size);
char *request_read_string(WbRequest *);  // to be released with g_free
bool request_is_over(WbRequest *);
int request_get_size(WbRequest *);
int request_get_position(const WbRequest *);
void request_set_position(WbRequest *, int pos);
void request_set_immediate(WbRequest *, bool immediate);
bool request_is_immediate(const WbRequest *);
void request_print(FILE *fd, WbRequest *);  // for debug

#define request_read_char(r) ((r)->data[(r)->pointer++])
#define request_read_uchar(r) (*((unsigned char *)&((r)->data[(r)->pointer++])))

#endif  // REQUEST_H
