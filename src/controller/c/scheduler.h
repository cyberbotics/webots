/*
 * Copyright 1996-2022 Cyberbotics Ltd.
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

// last modification: 19.12.01 by Yann
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "request.h"

#define scheduler_read_int32(d) (*((int *)(d)))

extern unsigned int scheduler_data_size;
extern char *scheduler_date;
extern unsigned int scheduler_actual_step;
extern char *scheduler_protocol;

int scheduler_init(const char *pipe);
void scheduler_cleanup();
WbRequest *scheduler_read_data();
void scheduler_send_request(WbRequest *);
bool scheduler_is_local();
int scheduler_get_pipe_handle();

#endif  // SCHEDULER_HH
