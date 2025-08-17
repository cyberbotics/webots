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

// last modification: 19.12.01 by Yann
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "request.h"

#define scheduler_read_int32(d) (*((int *)(d)))
#define scheduler_read_short(d) (*((short *)(d)))
#define scheduler_read_char(d) (*((char *)(d)))

extern unsigned int scheduler_data_size;
extern char *scheduler_date;
extern unsigned int scheduler_actual_step;

int scheduler_init_remote(const char *host, int port, const char *robot_name, char *buffer);
int scheduler_init_local(const char *pipe);
void scheduler_cleanup();
void scheduler_send_request(const WbRequest *);
WbRequest *scheduler_read_data();
WbRequest *scheduler_read_data_remote();
WbRequest *scheduler_read_data_local();
int scheduler_receive_meta(int pointer, size_t type_size);
int scheduler_receive_data(int pointer, int chunk_size);
void scheduler_receive_image(const unsigned char *buffer, int size);
bool scheduler_is_ipc();
bool scheduler_is_tcp();
int scheduler_get_pipe_handle();

#endif  // SCHEDULER_HH
