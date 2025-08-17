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

#include "robot_window_private.h"
#include "scheduler.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dynamic_library.h"

typedef void (*_V_V)(void);
typedef bool (*_B_V)(void);
typedef void *(*_VS_VS)(void *);
typedef unsigned long long (*_W_V)(void);

static DYNAMIC_LIBRARY_HANDLE library_handle = NULL;
static bool initialized = false;
static bool first_show = true;

static _B_V wbw_init = NULL;
static _V_V wbw_cleanup = NULL;
static _V_V wbw_pre_update_gui = NULL;
static _V_V wbw_update_gui = NULL;
static _V_V wbw_read_sensors = NULL;
static _V_V wbw_write_actuators = NULL;
static _V_V wbw_show = NULL;

void robot_window_init(const char *library_name) {
  if (initialized) {
    fprintf(stderr, "Error: robot window already initialized\n");
    return;
  }

  if (!library_name || strlen(library_name) < 1) {
    fprintf(stderr, "Error: robot window invalid library name\n");
    return;
  }

  // load the library
  library_handle = dynamic_library_init(library_name);
  if (library_handle == NULL) {
    fprintf(stderr, "Error: robot window initialization failed\n");
    robot_window_cleanup();
    return;
  }

  // get the entry points
  wbw_init = (_B_V)dynamic_library_get_symbol(library_handle, "wbw_init");
  wbw_cleanup = (_V_V)dynamic_library_get_symbol(library_handle, "wbw_cleanup");
  wbw_pre_update_gui = (_V_V)dynamic_library_get_symbol(library_handle, "wbw_pre_update_gui");
  wbw_update_gui = (_V_V)dynamic_library_get_symbol(library_handle, "wbw_update_gui");
  wbw_read_sensors = (_V_V)dynamic_library_get_symbol(library_handle, "wbw_read_sensors");
  wbw_write_actuators = (_V_V)dynamic_library_get_symbol(library_handle, "wbw_write_actuators");
  wbw_show = (_V_V)dynamic_library_get_symbol(library_handle, "wbw_show");
  if (!wbw_init) {
    fprintf(stderr, "Error: cannot find wbw_init entry point in robot window library\n");
    robot_window_cleanup();
    return;
  }
  if (!wbw_cleanup) {
    fprintf(stderr, "Error: cannot find wbw_cleanup entry point in robot window library\n");
    robot_window_cleanup();
    return;
  }
  if (!wbw_update_gui) {
    fprintf(stderr, "Error: cannot find wbw_update_gui entry point in robot window library\n");
    robot_window_cleanup();
    return;
  }

  // ok
  first_show = true;
  initialized = true;
}

void robot_window_read_sensors() {
  if (wbw_read_sensors)
    wbw_read_sensors();
}

void robot_window_write_actuators() {
  if (wbw_write_actuators)
    wbw_write_actuators();
}

void robot_window_pre_update_gui() {
  if (wbw_pre_update_gui)
    wbw_pre_update_gui();
}

void robot_window_update_gui() {
  if (wbw_update_gui)
    wbw_update_gui();
}

void robot_window_cleanup() {
  if (initialized) {
    if (wbw_cleanup)
      wbw_cleanup();
    dynamic_library_cleanup(library_handle);
    library_handle = NULL;
  }
  wbw_init = NULL;
  wbw_cleanup = NULL;
  wbw_pre_update_gui = NULL;
  wbw_update_gui = NULL;
  wbw_read_sensors = NULL;
  wbw_write_actuators = NULL;
  wbw_show = NULL;
  first_show = true;
  initialized = false;
}

void robot_window_show() {
  if (first_show) {
    first_show = false;
    if (wbw_init) {
      bool ret = wbw_init();
      if (!ret) {
        fprintf(stderr, "Error: robot window wbw_init() call failed\n");
        fflush(stderr);
        robot_window_cleanup();
        return;
      }
    }
  }

  if (wbw_show)
    wbw_show();
}

bool robot_window_is_initialized() {
  return initialized;
}

void *wb_robot_window_custom_function(void *args) {
  if (!initialized) {
    fprintf(stderr, "Error: the robot window is not initialized\n");
    fflush(stderr);
    return NULL;
  }

  _VS_VS foo = (_VS_VS)dynamic_library_get_symbol(library_handle, "wbw_robot_window_custom_function");
  if (!foo) {
    fprintf(stderr, "Error: wbw_robot_window_custom_function is not defined\n");
    fflush(stderr);
    return NULL;
  }

  return foo(args);
}
