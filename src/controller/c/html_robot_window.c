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

// This module managemes the shared library associated with an HTML robot
// window.
// Note: this file should be renamed to robot_window.c when the Qt robot
// windows are disabled.

#include <webots/utils/system.h>
#include "dynamic_library.h"
#include "html_robot_window_private.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static void (*wb_robot_window_init)(void) = NULL;
static void (*wb_robot_window_step)(int) = NULL;
static void (*wb_robot_window_cleanup)(void) = NULL;

static DYNAMIC_LIBRARY_HANDLE library_handle = NULL;

static void wb_robot_window_unload_library() {
  wb_robot_window_step = NULL;
  dynamic_library_cleanup(library_handle);
  library_handle = NULL;
}

bool wb_robot_window_load_library(const char *name) {
  if (name[0] == '\0')
    return false;
  // search for the corresponding HTML file, e.g., /fullpath/e-puck.dll => /fullpath/e-puck.html
  // On windows,  the name is /fullpath/e-puck.dll
  // On Linux,    the name is /fullpath/libe-puck.so
  // on macOS,    the name is /fullpath/libe-puck.dylib
  int l = strlen(name);
  assert(l > 3);
  char *html = (char *)malloc(l + 6);
  memcpy(html, name, l + 1);
#ifndef _WIN32  // we have to remove the lib prefix
  int slash;
  for (slash = l; html[slash] != '/' && slash >= 0; slash--) {
  }
  if (slash < 0) {
    free(html);
    return true;
  }
  l -= 3;
  int i;
  for (i = slash + 1; i < l; i++)
    html[i] = html[i + 3];
  html[i] = '\0';
#endif
  int dot;
  for (dot = l; dot > 0 && html[dot] != '.'; dot--) {
  }
  html[dot + 1] = 'h';
  html[dot + 2] = 't';
  html[dot + 3] = 'm';
  html[dot + 4] = 'l';
  html[dot + 5] = '\0';
  // if the corresponding HTML file doesn't exists, it is an old Qt robot window
  // and we don't want to open it here
  if (access(wbu_system_short_path(html), F_OK) == -1) {
    free(html);
    return true;
  }
  free(html);
  library_handle = dynamic_library_init(name);
  if (!library_handle) {
    fprintf(stderr, "Error: failed to load %s library\n", name);
    return false;
  }
  wb_robot_window_init = (void (*)())dynamic_library_get_symbol(library_handle, "wb_robot_window_init");
  wb_robot_window_step = (void (*)(int))dynamic_library_get_symbol(library_handle, "wb_robot_window_step");
  wb_robot_window_cleanup = (void (*)())dynamic_library_get_symbol(library_handle, "wb_robot_window_cleanup");
  if (!wb_robot_window_step) {
    wb_robot_window_unload_library();
    fprintf(stderr, "Error: cannot find any 'void wb_robot_window_step(int)' function in the %s robot window library\n", name);
    return false;
  }
  return true;
}

void html_robot_window_init() {
  if (wb_robot_window_init)
    (*wb_robot_window_init)();
}

void html_robot_window_step(int step) {
  if (wb_robot_window_step)
    (*wb_robot_window_step)(step);
}

void html_robot_window_cleanup() {
  if (wb_robot_window_cleanup)
    (*wb_robot_window_cleanup)();
}
