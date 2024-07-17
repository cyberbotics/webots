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

#include "g_pipe.h"

#ifdef _WIN32
#include <assert.h>
#include <fcntl.h>
#include <windows.h>
#else
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#endif  // _WIN32

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <webots/types.h>
#include "scheduler.h"

GPipe *g_pipe_new(const char *path) {
  GPipe *p = malloc(sizeof(GPipe));
#ifdef _WIN32
  p->fd[0] = 0;
  p->fd[1] = 0;
  while (1) {
    p->handle = CreateFile(path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (p->handle != INVALID_HANDLE_VALUE)
      break;
    DWORD dwError = GetLastError();
    if (dwError != ERROR_PIPE_BUSY) {
      free(p);
      return NULL;
    }
    if (!WaitNamedPipe(path, 5000)) {
      fprintf(stderr, "Cannot open pipe file: %s after trying for 5 seconds\n", path);
      free(p);
      return NULL;
    }
  }
#else
  p->handle = socket(PF_UNIX, SOCK_STREAM, 0);
  if (p->handle < 0) {
    fprintf(stderr, "socket() failed\n");
    // cppcheck-suppress memleak ; otherwise cppcheck shows a false positive for p->handle
    free(p);
    return NULL;
  }
  struct sockaddr_un address;
  memset(&address, 0, sizeof(struct sockaddr_un));
  address.sun_family = AF_UNIX;
  strncpy(address.sun_path, path, sizeof(address.sun_path));
  if (connect(p->handle, (struct sockaddr *)&address, sizeof(struct sockaddr_un)) != 0) {
    close(p->handle);
    free(p);
    return NULL;
  }
#endif
  return p;
}

void g_pipe_delete(GPipe *p) {
  if (p == NULL)
    return;
  if (p->handle)
#ifdef _WIN32
    CloseHandle(p->handle);
#else
    close(p->handle);
#endif
  free(p);
}

static void broken_pipe() {
  exit(1);
}

void g_pipe_send(GPipe *p, const char *data, int size) {
#ifdef _WIN32
  assert(p->handle);
  DWORD m = 0;
  if (WriteFile(p->handle, data, size, &m, NULL) == 0)
    broken_pipe();
#else
  int fd = p->handle;
  if (!fd)
    fd = p->fd[1];
  if (write(fd, data, size) == -1)
    broken_pipe();
#endif
}

int g_pipe_receive(GPipe *p, char *data, int size) {
#ifdef _WIN32
  DWORD nb_read = 0;
  DWORD e = ERROR_SUCCESS;
  BOOL success = false;
  assert(p->handle);
  do {
    success = ReadFile(p->handle, data, size, &nb_read, NULL);
    if (!success) {
      e = GetLastError();
      if (e != ERROR_MORE_DATA)
        break;
      e = ERROR_SUCCESS;
    }
  } while (!success);  // repeat loop while ERROR_MORE_DATA
  if (e != ERROR_SUCCESS)
    broken_pipe();
  return (int)nb_read;
#else
  int fd = p->handle;
  if (!fd)
    fd = p->fd[0];
  int n = read(fd, data, size);
  // the read function returns -1 when a debugger
  // is trying to attach to the process.
  // (The controller is very frequently at this step,
  //  for example, when the simulator is paused)
  // A second try is required in this case
  if (n == -1 && errno == EINTR)
    n = read(fd, data, size);
  if (n <= 0)
    broken_pipe();
  return n;
#endif
}

size_t g_pipe_get_handle(const GPipe *p) {
  if (p)
    return (size_t)p->handle;
  return 0;
}
