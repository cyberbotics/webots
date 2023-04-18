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

#ifndef MOUSE_PRIVATE_H
#define MOUSE_PRIVATE_H

#include "request.h"

void mouse_write_request(WbRequest *req);
void mouse_set_sampling_period(int sampling_period);
bool mouse_read_answer(int message, WbRequest *r);
void wb_mouse_init();

#endif  // MOUSE_PRIVATE_H
