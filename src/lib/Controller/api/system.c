/*
 * Copyright 1996-2019 Cyberbotics Ltd.
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

#include <stdio.h>
#include <webots/utils/system.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <stdlib.h>
#endif

#ifdef _WIN32
static char *buffer = NULL;

static void free_buffer() {
  free(buffer);
}
#endif

const char *wbu_system_getenv(const char *variable) {
#ifdef _WIN32
  wchar_t *value;
  size_t size = strlen(variable) + 1;
  wchar_t *wvariable = (wchar_t *)malloc(size * sizeof(wchar_t));
  MultiByteToWideChar(CP_UTF8, 0, variable, -1, wvariable, size);
  size = GetEnvironmentVariableW(wvariable, NULL, 0);
  if (size == 0)
    return NULL;  // not defined
  value = (wchar_t *)malloc(size * sizeof(wchar_t));
  GetEnvironmentVariableW(wvariable, value, size);
  free(wvariable);
  size = WideCharToMultiByte(CP_UTF8, 0, value, -1, NULL, 0, NULL, NULL);
  if (buffer == NULL)
    atexit(free_buffer);
  else
    free(buffer);
  buffer = (char *)malloc(size);
  WideCharToMultiByte(CP_UTF8, 0, value, -1, buffer, size, NULL, NULL);
  free(value);
  return (const char *)buffer;
#else
  return getenv(variable);
#endif
}

const char *wbu_system_short_path(const char *path) {
#ifdef _WIN32
  int size = MultiByteToWideChar(CP_UTF8, 0, path, -1, NULL, 0);
  wchar_t *w_path = (wchar_t *)malloc(size * sizeof(wchar_t));
  MultiByteToWideChar(CP_UTF8, 0, path, -1, w_path, size);
  size = GetShortPathNameW(w_path, NULL, 0);
  wchar_t *w_short_path = (wchar_t *)malloc(size * sizeof(wchar_t));
  GetShortPathNameW(w_path, w_short_path, size);
  free(w_path);
  size = WideCharToMultiByte(CP_UTF8, 0, w_short_path, -1, NULL, 0, NULL, NULL);
  if (buffer == NULL)
    atexit(free_buffer);
  else
    free(buffer);
  buffer = (char *)malloc(size);
  WideCharToMultiByte(CP_UTF8, 0, w_short_path, -1, buffer, size, NULL, NULL);
  return (const char *)buffer;
#else
  return path;
#endif
}
