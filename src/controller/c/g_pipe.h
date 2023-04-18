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

#ifndef G_PIPE_H
#define G_PIPE_H

#ifdef _WIN32
#include <windows.h>
#endif  // _WIN32
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

struct _GPipe {
  int fd[2];
#ifdef _WIN32
  HANDLE handle;
  char *buffer;
  int pointer;
  int read;
  int buffer_size;
  int fd_local[2];  // for the local pipe
#else
  int handle;
#endif
};

typedef struct _GPipe GPipe;

GPipe *g_pipe_new(const char *);  // named pipe on Windows, UNIX domain socket on Linux / macOS
void g_pipe_delete(GPipe *);
void g_pipe_send(GPipe *, const char *data, int size);
int g_pipe_receive(GPipe *, char *data, int size);
size_t g_pipe_get_handle(GPipe *);

#ifdef __cplusplus
}
#endif

#endif  // G_PIPE_H
