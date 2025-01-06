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

#ifndef DYNAMIC_LIBRARY_H
#define DYNAMIC_LIBRARY_H

#include <stdlib.h>
#include <webots/types.h>

#ifdef _WIN32
struct HINSTANCE__;
typedef struct HINSTANCE__ *dlInstance;
#define DYNAMIC_LIBRARY_HANDLE dlInstance
#else  // __linux__ || __APPLE__
#define DYNAMIC_LIBRARY_HANDLE void *
#endif

DYNAMIC_LIBRARY_HANDLE dynamic_library_init(const char *libname);
bool dynamic_library_is_loaded(DYNAMIC_LIBRARY_HANDLE);
void dynamic_library_cleanup(DYNAMIC_LIBRARY_HANDLE);
void *dynamic_library_get_symbol(DYNAMIC_LIBRARY_HANDLE, const char *symbol);
const char *dynamic_library_get_last_error();

#endif  // DYNAMIC_LIBRARY_H
