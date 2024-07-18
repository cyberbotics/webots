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

// generic robot controller API
//
// this module includes:
// (1) robot initialization
// (2) management of devices (device list, requests, answers)
// (3) handling basic robot requests
// (4) initialization of the remote scene if any (textures, download)

#include <assert.h>
#include <dirent.h>  // struct dirent
#include <fcntl.h>
#include <locale.h>  // LC_NUMERIC
#include <signal.h>  // signal
#include <stdarg.h>
#include <stdio.h>     // snprintf
#include <stdlib.h>    // exit
#include <string.h>    // strlen
#include <sys/stat.h>  // stat
#include <unistd.h>    // sleep, pipe, dup2, STDOUT_FILENO, STDERR_FILENO

#if defined(__APPLE__) || defined(_WIN32)
#define st_mtim st_mtimespec
#endif

#include <webots/console.h>
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/mouse.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/types.h>
#include <webots/utils/system.h>

#include "default_robot_window_private.h"
#include "device_private.h"
#include "html_robot_window_private.h"
#include "joystick_private.h"
#include "keyboard_private.h"
#include "messages.h"
#include "motion_private.h"
#include "mouse_private.h"
#include "percent.h"
#include "remote_control_private.h"
#include "request.h"
#include "robot_private.h"
#include "robot_window_private.h"
#include "scheduler.h"
#include "sha1.h"
#include "supervisor_private.h"
#include "tcp_client.h"

#ifdef _WIN32
#include <windows.h>  // GetCommandLine
#else
#include <pthread.h>
#endif

#define WEBOTS_EXIT_FALSE 0
#define WEBOTS_EXIT_NOW 1
#define WEBOTS_EXIT_LATER 2

#ifndef LIBCONTROLLER_VERSION
#error "Missing declaration of LIBCONTROLLER_VERSION preprocessor macro in the Makefile."
#endif

typedef struct {
  WbDevice **device;  // array of devices
  double battery_value;
  void (*real_robot_cleanup)(void);
  int battery_sampling_period;
  int n_device;  // number of devices, including the robot itself
  bool is_supervisor;
  unsigned char synchronization;
  WbRobotMode mode;
  unsigned char configure;  // 1 if just configured: reset need to be called
  unsigned char client_exit;
  unsigned char webots_exit;  // WEBOTS_EXIT_FALSE, WEBOTS_EXIT_NOW or WEBOTS_EXIT_LATER
  double basic_time_step;
  char *console_stdout;
  char *console_stderr;
  char *project_path;
  char *world_path;
  char *model;
  char *window_filename;
  char *remote_control_filename;
  char *controller_name;
  char *urdf;
  bool need_urdf;
  char *urdf_prefix;
  char *custom_data;
  bool is_immediate_message;
  bool is_waiting_for_user_input_event;
  WbUserInputEvent user_input_event_type;
  int user_input_event_timeout;
  bool dataNeedToWriteRequest;
  bool show_window;
  bool has_html_robot_window;
  bool update_window;
  bool toggle_remote_first_step;  // true in the first step after a switch between remote/simulation
  bool send_remote_mode_off;      // tell to send the REMOTE_OFF message in the next robot_write_request()
  int pin;
  int wwi_message_to_send_size;
  const char *wwi_message_to_send;
  int wwi_received_messages_size;
  int wwi_reception_buffer_size;
  char *wwi_reception_buffer;
  bool wwi_reset_reading_head;
  WbSimulationMode simulation_mode;  // WB_SUPERVISOR_SIMULATION_MODE_FAST, etc.
} WbRobot;

static char *WEBOTS_VERSION;
static bool robot_init_was_done = false;
static WbRobot robot;
static WbMutexRef robot_step_mutex;
static double simulation_time = 0.0;
static unsigned int current_step_duration = 0;
static bool should_abort_simulation_waiting = false;
static bool waiting_for_step_begin = false;
static bool waiting_for_step_end = false;
static int stdout_read = -1;
static int stderr_read = -1;

// Static functions
static int stream_pipe_create(int stream) {
  int fds[2];
#ifdef WIN32
  _pipe(fds, 1024, O_TEXT);
#else
  if (pipe(fds) == -1) {
    fprintf(stderr, "Error: cannot create pipe for WEBOTS_STDOUT_REDIRECT.\n");
    exit(EXIT_FAILURE);
  }
  fcntl(fds[0], F_SETFL, O_NONBLOCK);
#endif
  dup2(fds[1], stream);
  return fds[0];
}

void stream_pipe_read(int fd, char **buffer) {
  if (fd == -1)
    return;
  assert(*buffer == NULL);
  *buffer = malloc(1024);  // FIXME: buffer is limited to 1024 bytes
#ifdef _WIN32
  int len = eof(fd) ? 0 : read(fd, *buffer, 1023);
#else
  int len = read(fd, *buffer, 1023);
  if (len == -1)
    len = 0;
#endif
  if (len != 0)
    (*buffer)[len] = '\0';
  else {
    free(*buffer);
    *buffer = NULL;
  }
}

static void init_robot_window_library() {
  if (robot_window_is_initialized())
    return;

  robot_window_init(robot.window_filename);
  if (!robot_window_is_initialized())
    fprintf(stderr, "Error: Cannot load the \"%s\" robot window library.\n", robot.window_filename);
}

static void quit_controller(int signal_number) {
  should_abort_simulation_waiting = true;
  signal(signal_number, SIG_DFL);
  raise(signal_number);
}

static void init_remote_control_library() {
  if (remote_control_is_initialized())
    return;

  if (strlen(robot.remote_control_filename) > 0) {
    remote_control_init(robot.remote_control_filename);
    if (!remote_control_is_initialized())
      fprintf(stderr, "Error: Cannot load the \"%s\" remote control library.\n", robot.remote_control_filename);
  }
}

static void init_devices_from_tag(WbRequest *r, int firstTag) {
  for (int tag = firstTag; tag < robot.n_device; tag++) {
    robot.device[tag] = malloc(sizeof(WbDevice));
    robot.device[tag]->node = request_read_uint16(r);
    robot.device[tag]->name = request_read_string(r);
    robot.device[tag]->model = request_read_string(r);
    // printf("reading %s (%d) (%s)\n", robot.device[tag]->name, robot.device[tag]->node, robot.device[tag]->model);
    wb_device_init(robot.device[tag]);  // set device specific fields (read_answer and device_data)
  }
}

static void robot_quit() {  // called when Webots kills a controller
  WbDeviceTag tag;
  for (tag = 0; tag < robot.n_device; tag++)
    wb_device_cleanup(robot.device[tag]);
  free(robot.device);
  robot.device = NULL;
  robot.n_device = 0;
  if (robot.real_robot_cleanup)
    (*(robot.real_robot_cleanup))();
  scheduler_cleanup();
  motion_cleanup();
  free(robot.project_path);
  robot.project_path = NULL;
  free(robot.world_path);
  robot.world_path = NULL;
  free(robot.model);
  robot.model = NULL;
  free(robot.window_filename);
  robot.window_filename = NULL;
  free(robot.controller_name);
  robot.controller_name = NULL;
  free(robot.custom_data);
  robot.custom_data = NULL;
  free(robot.console_stdout);
  robot.console_stdout = NULL;
  free(robot.console_stderr);
  robot.console_stderr = NULL;
  free(robot.wwi_reception_buffer);
  robot.wwi_reception_buffer = NULL;
  robot_window_cleanup();
  remote_control_cleanup();
  free(robot.urdf);
  free(robot.urdf_prefix);
}

// this function is also called from supervisor_write_request()
void robot_write_request(WbDevice *dev, WbRequest *req) {
  keyboard_write_request(req);
  joystick_write_request(req);
  mouse_write_request(req);
  if (robot.battery_value < 0.0) {  // need to enable or disable
    request_write_uchar(req, C_ROBOT_SET_BATTERY_SAMPLING_PERIOD);
    request_write_uint16(req, robot.battery_sampling_period);
    robot.battery_value = 0.0;
  }
  if (robot.dataNeedToWriteRequest) {  // need to send new robot data
    request_write_uchar(req, C_ROBOT_SET_DATA);
    request_write_uint16(req, strlen(robot.custom_data) + 1);
    request_write_string(req, robot.custom_data);
    robot.dataNeedToWriteRequest = false;
  }
  if (robot.console_stdout && robot.console_stdout[0]) {
    request_write_uchar(req, C_CONSOLE_MESSAGE);
    request_write_uchar(req, 1);
    request_write_uint32(req, strlen(robot.console_stdout) + 1);
    request_write_string(req, robot.console_stdout);
    free(robot.console_stdout);
    robot.console_stdout = NULL;
  }
  if (robot.console_stderr && robot.console_stderr[0]) {
    request_write_uchar(req, C_CONSOLE_MESSAGE);
    request_write_uchar(req, 2);
    request_write_uint32(req, strlen(robot.console_stderr) + 1);
    request_write_string(req, robot.console_stderr);
    free(robot.console_stderr);
    robot.console_stderr = NULL;
  }
  if (robot.client_exit) {
    request_write_uchar(req, C_ROBOT_CLIENT_EXIT_NOTIFY);
  }
  if (robot.send_remote_mode_off == true) {
    robot.send_remote_mode_off = false;
    request_write_uchar(req, C_ROBOT_REMOTE_OFF);
  }
  if (robot.pin >= 0) {
    request_write_uchar(req, C_ROBOT_PIN);
    request_write_uchar(req, robot.pin != 0);
    robot.pin = -1;
  }
  if (robot.wwi_message_to_send_size) {
    request_write_uchar(req, C_ROBOT_WWI_MESSAGE);
    request_write_int32(req, robot.wwi_message_to_send_size);
    request_write_data(req, robot.wwi_message_to_send, robot.wwi_message_to_send_size);
    robot.wwi_message_to_send_size = 0;
  }
  if (robot.is_waiting_for_user_input_event) {
    request_write_uchar(req, C_ROBOT_WAIT_FOR_USER_INPUT_EVENT);
    request_write_int32(req, robot.user_input_event_type);
    request_write_int32(req, robot.user_input_event_timeout);
  }
  if (robot.need_urdf) {
    request_write_uchar(req, C_ROBOT_URDF);
    request_write_uint16(req, strlen(robot.urdf_prefix) + 1);
    request_write_string(req, robot.urdf_prefix);
  }
}

static WbRequest *generate_request(unsigned int step_duration, bool toggle_remote) {
  WbRequest *req = request_new_empty();
  request_write_uint32(req, step_duration);

  WbDeviceTag tag;
  for (tag = 0; tag < robot.n_device; tag++) {
    int begin = request_get_position(req);
    request_write_tag(req, tag);
    request_write_int32(req, 0);  // reserve 1 int for the size
    int before = request_get_position(req);
    if (toggle_remote && robot.device[tag]->toggle_remote)
      robot.device[tag]->toggle_remote(robot.device[tag], req);
    else
      robot.device[tag]->write_request(robot.device[tag], req);

    int after = request_get_position(req);
    int size = after - before;  // calc number of bytes in this request
    if (size > 0) {
      // write only if not empty
      request_set_position(req, before - sizeof(int));
      request_write_int32(req, size);  // now write size in reserved int
      request_set_position(req, after);
    } else
      // reset to beginning -> 0 bytes will be sent
      request_set_position(req, begin);
  }
  request_write_size(req);
  return req;
}

static void robot_send_request(unsigned int step_duration) {
  WbRequest *req = generate_request(step_duration, false);

  if (wb_robot_get_mode() == WB_MODE_REMOTE_CONTROL) {
    // it returns a partial request copy to be sent to Webots.
    WbRequest *copy = remote_control_handle_messages(req);
    request_delete(req);
    req = copy;
    remote_control_step(step_duration);
  }

  if (scheduler_is_ipc() || scheduler_is_tcp() || request_get_size(req) != 8)
    scheduler_send_request(req);
  request_delete(req);
}

// rebuild the device list
static void robot_configure(WbRequest *r) {
  free(robot.device[0]->name);
  robot.device[0]->name = request_read_string(r);

  WEBOTS_VERSION = request_read_string(r);
  if (strlen(WEBOTS_VERSION) && (strncmp(WEBOTS_VERSION, LIBCONTROLLER_VERSION, 6)))
    fprintf(stderr,
            "Warning: Webots [%s] and libController [%s] versions are not the same for Robot '%s'! Different versions can lead "
            "to undefined behavior.\n",
            WEBOTS_VERSION, LIBCONTROLLER_VERSION, robot.device[0]->name);

  // delete all the devices except the robot
  WbDeviceTag tag;
  for (tag = 1; tag < robot.n_device; tag++)
    wb_device_cleanup(robot.device[tag]);

  // read the supervisor
  robot.is_supervisor = request_read_uchar(r);
  // read the synchronization
  robot.synchronization = request_read_uchar(r);
  // reading number of devices
  robot.n_device = request_read_uint16(r);
  ROBOT_ASSERT(robot.n_device > 0);
  WbDevice *d = robot.device[0];  // save pointer to the root device
  free(robot.device);
  robot.device = malloc(sizeof(WbDevice *) * robot.n_device);
  robot.device[0] = d;  // restore pointer to root device
  robot.device[0]->node = request_read_uint16(r);
  simulation_time = request_read_double(r);

  // printf("robot.is_supervisor = %d\n", robot.is_supervisor);
  // printf("robot.synchronization = %d\n", robot.synchronization);
  // printf("robo.n_device = %d\n", robot.n_device);
  // printf("robot.device[0]->node = %d\n", robot.device[0]->node);
  // printf("robot.device[0]->name = %s\n", robot.device[0]->name);

  switch (robot.device[0]->node) {
    case WB_NODE_ROBOT:
      if (robot.is_supervisor)
        wb_supervisor_init(robot.device[0]);
      break;
    default:
      ROBOT_ASSERT(0);
      break;
  }
  // reading device names
  init_devices_from_tag(r, 1);

  robot.configure = 1;
  robot.basic_time_step = request_read_double(r);
  robot.project_path = request_read_string(r);
  robot.world_path = request_read_string(r);
  robot.model = request_read_string(r);
  robot.window_filename = request_read_string(r);
  robot.remote_control_filename = request_read_string(r);
  robot.controller_name = request_read_string(r);
  robot.custom_data = request_read_string(r);
  robot.show_window = request_read_uchar(r);
  robot.has_html_robot_window = request_read_uchar(r);
  wb_robot_window_load_library(robot.window_filename);
  robot.simulation_mode = request_read_int32(r);
  robot.is_immediate_message = false;
  // printf("configure done\n");
}

static char robot_read_data() {
  bool immediate = false;

  do {
    robot_window_update_gui();

    WbRequest *r = scheduler_read_data();
    while (r == NULL) {
      fprintf(stderr, "Warning: %s(): received empty data request!\n", __FUNCTION__);
      r = scheduler_read_data();
    }
    while (request_is_over(r) == false) {
      WbDeviceTag tag = request_read_tag(r);
      // fprintf(stderr, "robot_read_data: tag=%d\n", tag);
      ROBOT_ASSERT(tag < robot.n_device);
      WbDevice *dev = robot.device[tag];
      dev->read_answer(dev, r);
    }

    immediate = request_is_immediate(r);

    request_delete(r);

    if (immediate) {
      if (robot.show_window && !robot.has_html_robot_window) {  // Qt-based robot window
        // initialize first the remote control library in order
        // to have access to the real robot at the window creation
        // to get custom data
        init_robot_window_library();
        robot_window_show();
        robot.show_window = false;
      }
      if (robot.update_window) {  // HTML robot window
        html_robot_window_step(0);
        robot.update_window = false;
      }
      robot_window_pre_update_gui();
    }

  } while (immediate);

  return true;
}

void robot_read_answer(WbDevice *d, WbRequest *r) {
  int n;
  int message = request_read_uchar(r);

  if (keyboard_read_answer(message, r))
    return;

  if (joystick_read_answer(message, r))
    return;

  if (mouse_read_answer(message, r))
    return;

  switch (message) {
    case C_ROBOT_TIME:
      simulation_time = request_read_double(r);
      break;
    case C_CONFIGURE:
      robot_configure(r);
      break;
    case C_ROBOT_BATTERY_VALUE:
      // printf("received robot battery value\n");
      robot.battery_value = request_read_double(r);
      break;
    case C_ROBOT_DATA:
      free(robot.custom_data);
      robot.custom_data = request_read_string(r);
      break;
    case C_ROBOT_SUPERVISOR:
      robot.is_supervisor = request_read_uchar(r);
      break;
    case C_ROBOT_MODEL:
      free(robot.model);
      robot.model = request_read_string(r);
      break;
    case C_ROBOT_NEW_DEVICE:
      n = request_read_uint16(r);
      robot.device = realloc(robot.device, sizeof(WbDevice *) * (robot.n_device + n));
      if (!robot.device) {
        fprintf(stderr, "Error initializing the new device: not enough memory.\n");
        exit(EXIT_FAILURE);
      }
      const int firstTag = robot.n_device;
      robot.n_device += n;
      init_devices_from_tag(r, firstTag);
    case C_ROBOT_WINDOW_SHOW:
      robot.show_window = true;
      break;
    case C_ROBOT_WINDOW_UPDATE:
      robot.update_window = true;
      break;
    case C_ROBOT_WWI_MESSAGE:
      n = request_read_int32(r);
      const int new_size = robot.wwi_received_messages_size + n;
      if (robot.wwi_reception_buffer_size < new_size) {
        robot.wwi_reception_buffer = realloc(robot.wwi_reception_buffer, new_size);
        robot.wwi_reception_buffer_size = new_size;
      }
      memcpy(robot.wwi_reception_buffer + robot.wwi_received_messages_size, request_read_data(r, n), n);
      robot.wwi_received_messages_size += n;
      break;
    case C_ROBOT_SIMULATION_CHANGE_MODE:
      robot.simulation_mode = request_read_int32(r);
      if (robot.simulation_mode == WB_SUPERVISOR_SIMULATION_MODE_PAUSE && wb_robot_get_mode() == WB_MODE_REMOTE_CONTROL)
        remote_control_stop_actuators();
      break;
    case C_ROBOT_QUIT:
      robot.webots_exit = WEBOTS_EXIT_NOW;
      break;
    case C_ROBOT_WAIT_FOR_USER_INPUT_EVENT:
      robot.is_waiting_for_user_input_event = false;
      robot.user_input_event_type = request_read_int32(r);
      break;
    case C_ROBOT_URDF:
      free(robot.urdf);
      robot.urdf = request_read_string(r);
      break;
    default:
      r->pointer--;  // unread the char from the request
      break;
  }
}

// Protected functions available from other files of the client library

const char *robot_get_device_name(WbDeviceTag tag) {
  if (tag < robot.n_device)
    return robot.device[tag]->name;

  return NULL;
}

const char *robot_get_device_model(WbDeviceTag tag) {
  if (tag < robot.n_device)
    return robot.device[tag]->model;

  return NULL;
}

int robot_get_number_of_devices() {
  return robot.n_device;
}

WbDevice *robot_get_robot_device() {
  return robot.device[0];
}

int robot_check_supervisor(const char *func_name) {
  if (robot.is_supervisor)
    return 1;  // OK

  fprintf(stderr, "Error: ignoring illegal call to %s() in a 'Robot' controller.\n", func_name);
  fprintf(stderr, "Error: this function can only be used in a 'Supervisor' controller.\n");
  return 0;
}

WbDevice *robot_get_device_with_node(WbDeviceTag tag, WbNodeType node, bool warning) {
  if (tag < robot.n_device) {  // exists
    WbDevice *d = robot.device[tag];
    if (d->node == node)
      return d;
    else
      return NULL;
  }
  if (warning)
    fprintf(stderr, "Error: device with tag=%d not found.\n", (int)tag);
  return NULL;
}

WbDevice *robot_get_device(WbDeviceTag tag) {
  if (tag < robot.n_device) {  // exists
    WbDevice *d = robot.device[tag];
    return d;
  }
  fprintf(stderr, "Error: device with tag=%d not found.\n", (int)tag);
  return NULL;
}

void wb_robot_cleanup() {  // called when the client quits
  html_robot_window_cleanup();
  default_robot_window_cleanup();

  if (robot.n_device == 0)
    return;

  if (wb_robot_get_mode() == WB_MODE_REMOTE_CONTROL)
    wb_robot_set_mode(0, NULL);

  robot.client_exit = true;
  wb_robot_step(0);
  robot_quit();
}

void robot_mutex_lock() {
  wb_robot_mutex_lock(robot_step_mutex);
}

void robot_mutex_unlock() {
  wb_robot_mutex_unlock(robot_step_mutex);
}

WbDeviceTag robot_get_device_tag(const WbDevice *d) {
  WbDeviceTag tag;
  for (tag = 0; tag < robot.n_device; tag++) {
    if (robot.device[tag] == d)
      return tag;
  }
  return 0;
}

void robot_abort(const char *format, ...) {
  va_list args;
  char message[1024];
  va_start(args, format);
  vsprintf(message, format, args);
  va_end(args);
  fprintf(stderr, "Abort: %s\n", message);
  robot_send_request(0);
  robot_read_data();
  exit(EXIT_FAILURE);
}

WbNodeType robot_get_device_type(WbDeviceTag tag) {
  if (tag < robot.n_device)
    return robot.device[tag]->node;
  return WB_NODE_NO_NODE;
}

int robot_is_quitting() {
  return robot.webots_exit;
}

void robot_toggle_remote(WbDevice *d, WbRequest *r) {
  if (robot.battery_sampling_period != 0)
    robot.battery_value = -1;

  if (wb_robot_get_mode() == WB_MODE_REMOTE_CONTROL)
    request_write_uchar(r, C_ROBOT_REMOTE_ON);
  else if (wb_robot_get_mode() == WB_MODE_SIMULATION)
    // in that case the request is not sent to webots
    // do it in the next robot_write_request
    robot.send_remote_mode_off = true;
}

void robot_console_print(const char *text, int stream) {
  const int n = strlen(text) + 1;
  if (stream == WB_STDOUT) {
    robot.console_stdout = malloc(n);
    memcpy(robot.console_stdout, text, n);
  } else if (stream == WB_STDERR) {
    robot.console_stderr = malloc(n);
    memcpy(robot.console_stderr, text, n);
  }
  if (wb_robot_step(0) == -1) {
    robot_quit();
    exit(EXIT_SUCCESS);
  }
}

bool robot_is_immediate_message() {
  return robot.is_immediate_message;
}

// Public functions available from the robot API

// multi-thread API

void wb_robot_task_new(void (*task)(void *), void *param) {  // create a task
#ifdef _WIN32
  DWORD thread_id;
  HANDLE thread_handle = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)task, param, 0, &thread_id);
  if (!thread_handle) {
    fprintf(stderr, "Error: %s() failed to create new thread.\n", __FUNCTION__);
    exit(EXIT_FAILURE);
  }
#else
  pthread_t thread;
  pthread_create(&thread, NULL, (void *(*)(void *))task, param);
#endif
  // cppcheck-suppress resourceLeak ; for thread_handle (which we don't need anymore)
}

WbMutexRef wb_robot_mutex_new() {
#ifdef _WIN32
  HANDLE m = CreateMutex(NULL, false, NULL);
#else
  // create a recursive mutex -> same thread can lock it multiple times
  pthread_mutexattr_t attributes;
  pthread_mutexattr_init(&attributes);
  pthread_mutexattr_settype(&attributes, PTHREAD_MUTEX_RECURSIVE);

  pthread_mutex_t *m = malloc(sizeof(pthread_mutex_t));
  pthread_mutex_init(m, &attributes);

  pthread_mutexattr_destroy(&attributes);
#endif
  // cppcheck-suppress resourceLeak
  return (WbMutexRef)m;
}

void wb_robot_mutex_lock(WbMutexRef m) {
// printf("lock\n");
#ifdef _WIN32
  WaitForSingleObject((HANDLE)m, INFINITE);
#else
  pthread_mutex_lock((pthread_mutex_t *)m);
#endif
}

void wb_robot_mutex_unlock(WbMutexRef m) {
// printf("unlock\n");
#ifdef _WIN32
  ReleaseMutex((HANDLE)m);
#else
  pthread_mutex_unlock((pthread_mutex_t *)m);
#endif
}

void wb_robot_mutex_delete(WbMutexRef m) {
#ifdef _WIN32
  CloseHandle((HANDLE)m);
#else
  pthread_mutex_destroy((pthread_mutex_t *)m);
  free(m);
#endif
}

// end of multi-task API

void wb_robot_set_mode(WbRobotMode mode, const char *arg) {
  if (mode != WB_MODE_SIMULATION && mode != WB_MODE_REMOTE_CONTROL) {
    fprintf(stderr, "Error: %s() cannot set mode to %d.\n", __FUNCTION__, mode);
    return;
  }
  if (robot.mode == WB_MODE_REMOTE_CONTROL && mode == WB_MODE_SIMULATION && remote_control_is_initialized()) {
    remote_control_stop();  // deactivate the remote control
    robot.toggle_remote_first_step = true;
  } else if (robot.mode == WB_MODE_SIMULATION && mode == WB_MODE_REMOTE_CONTROL &&
             remote_control_is_initialized()) {  // activate the remote control
    if (remote_control_start(arg)) {
      // remote control
      robot.mode = mode;
      robot.toggle_remote_first_step = true;
      return;
    } else
      fprintf(stderr, "Error: %s(): starting the remote control library (wbr_start) failed.\n", __FUNCTION__);
  }
  robot.mode = WB_MODE_SIMULATION;
}

const char *wb_robot_get_model() {
  return robot.model;
}

const char *wb_robot_get_custom_data() {
  return robot.custom_data;
}

void wb_robot_set_custom_data(const char *data) {
  free(robot.custom_data);
  const int size = strlen(data) + 1;
  robot.custom_data = malloc(size);
  memcpy(robot.custom_data, data, size);
  robot.dataNeedToWriteRequest = true;
}

const char *wb_robot_get_data() {
  return wb_robot_get_custom_data();
}

void wb_robot_set_data(const char *data) {
  wb_robot_set_custom_data(data);
}

int wb_robot_get_number_of_devices() {
  return robot.n_device - 1;  // the first item is the robot and not a device
}

WbDeviceTag wb_robot_get_device_by_index(int index) {
  if (index >= 0 && index < wb_robot_get_number_of_devices())
    return (WbDeviceTag)index + 1;  // the first item is the robot and not a device
  else {
    fprintf(stderr, "Error: The index of %s() is out of the bounds.\n", __FUNCTION__);
    return 0;
  }
}

WbDeviceTag wb_robot_get_device(const char *name) {
  if (!name || !name[0]) {
    fprintf(stderr, "Error: %s() called with NULL or empty argument.\n", __FUNCTION__);
    return 0;
  }

  if (!robot_init_was_done) {
    // we need to redirect the streams to make this message appear in the console
    wb_robot_init();
    robot_abort("wb_robot_init() must be called before any other Webots function.\n");
  }

  WbDeviceTag tag;
  for (tag = 0; tag < robot.n_device; tag++) {
    if (strcmp(robot.device[tag]->name, name) == 0)
      return tag;
  }

  fprintf(stderr, "Warning: \"%s\" device not found.\n", name);
  return 0;  // error: device not found (can not be the root device)
}

const char *wb_robot_get_name() {
  return robot.device[0]->name;
}

void wb_robot_battery_sensor_enable(int sampling_period) {
  robot_mutex_lock();
  robot.battery_value = -1.0;  // need to enable or disable
  robot.battery_sampling_period = sampling_period;
  robot_mutex_unlock();
}

void wb_robot_battery_sensor_disable() {
  wb_robot_battery_sensor_enable(0);
}

double wb_robot_battery_sensor_get_value() {
  if (robot.battery_sampling_period <= 0)
    fprintf(stderr, "Error: %s() called for a disabled device! Please use: wb_robot_battery_sensor_enable().\n", __FUNCTION__);
  double result;
  robot_mutex_lock();
  result = robot.battery_value;
  robot_mutex_unlock();
  return result;
}

int wb_robot_battery_sensor_get_sampling_period() {
  int sampling_period = 0;
  robot_mutex_lock();
  sampling_period = robot.battery_sampling_period;
  robot_mutex_unlock();
  return sampling_period;
}

void wbr_robot_battery_sensor_set_value(double value) {
  if (value < 0)
    fprintf(stderr, "Error: %s() received negative value, new value ignored.\n", __FUNCTION__);
  else
    robot.battery_value = value;
}

static int robot_step_begin(int duration) {
  if (waiting_for_step_end)
    fprintf(stderr, "Warning: %s() called multiple times before calling wb_robot_step_end().\n", __FUNCTION__);

  robot.wwi_reset_reading_head = true;

  if (!robot.client_exit)
    html_robot_window_step(duration);

  // transfer state to/from remote (but not when client exit)
  if (robot.toggle_remote_first_step && !robot.client_exit) {
    robot.toggle_remote_first_step = false;
    WbRequest *req = generate_request(0, true);
    if (wb_robot_get_mode() == WB_MODE_SIMULATION) {  // remote desactivated
      // nothing to do since the display is still simulated in remote mode.
      // remote_control_handle_messages(r);
    } else if (wb_robot_get_mode() == WB_MODE_REMOTE_CONTROL) {  // remote activated
      scheduler_send_request(req);
      robot_read_data();
    }
    request_delete(req);
  }

  current_step_duration = duration;

  if (robot.webots_exit == WEBOTS_EXIT_NOW) {
    robot_quit();
    robot_mutex_unlock();
    exit(EXIT_SUCCESS);
  } else if (robot.webots_exit == WEBOTS_EXIT_LATER) {
    robot.webots_exit = WEBOTS_EXIT_NOW;
    return -1;
  }

  robot_window_write_actuators();
  robot_window_pre_update_gui();

  motion_step_all(duration);
  robot_send_request(duration);

  waiting_for_step_begin = false;
  waiting_for_step_end = true;

  return 0;
}

static int robot_step_end() {
  if (waiting_for_step_begin)
    fprintf(stderr, "Warning: %s() called multiple times before calling wb_robot_step_begin().\n", __FUNCTION__);

  if (robot.webots_exit == WEBOTS_EXIT_NOW)
    return -1;

  keyboard_step_end();
  joystick_step_end();
  robot_read_data();

  int e = -1;
  if (robot.webots_exit == WEBOTS_EXIT_FALSE)
    e = scheduler_actual_step;

  if (e != -1 && wb_robot_get_mode() == WB_MODE_REMOTE_CONTROL && remote_control_has_failed())
    wb_robot_set_mode(0, NULL);

  robot_window_read_sensors();

  waiting_for_step_begin = true;
  waiting_for_step_end = false;

  return e;
}

int wb_robot_step_begin(int duration) {
  if (stdout_read != -1 || stderr_read != -1) {
    fflush(NULL);  // we need to flush the pipes
    stream_pipe_read(stdout_read, &(robot.console_stdout));
    stream_pipe_read(stderr_read, &(robot.console_stderr));
  }

  robot_mutex_lock();
  const int e = robot_step_begin(duration);
  robot_mutex_unlock();

  return e;
}

int wb_robot_step_end() {
  robot_mutex_lock();
  const int e = robot_step_end();
  robot_mutex_unlock();

  return e;
}

int wb_robot_step(int duration) {
  if (stdout_read != -1 || stderr_read != -1) {
    fflush(NULL);  // we need to flush the pipes
    stream_pipe_read(stdout_read, &(robot.console_stdout));
    stream_pipe_read(stderr_read, &(robot.console_stderr));
  }

  robot_mutex_lock();
  if (waiting_for_step_end)
    fprintf(stderr, "Warning: %s() called before calling wb_robot_step_end().\n", __FUNCTION__);
  int e = robot_step_begin(duration);
  if (e != -1)
    e = robot_step_end();
  robot_mutex_unlock();

  return e;
}

WbUserInputEvent wb_robot_wait_for_user_input_event(WbUserInputEvent event_type, int timeout) {
  // check that the devices associated with the event type are enabled
  bool valid = event_type == WB_EVENT_NO_EVENT;
  if (event_type & (WB_EVENT_MOUSE_CLICK | WB_EVENT_MOUSE_MOVE)) {
    if (wb_mouse_get_sampling_period() <= 0)
      fprintf(stderr,
              "Error: %s() called with an event type including the mouse, but the mouse is disabled, please enable it with "
              "wb_mouse_enable().\n",
              __FUNCTION__);
    else
      valid = true;
  }
  if (event_type & WB_EVENT_KEYBOARD) {
    if (wb_keyboard_get_sampling_period() <= 0)
      fprintf(stderr,
              "Error: %s() called with an event type including the keyboard, but the keyboard is disabled, please enable it "
              "with wb_keyboard_enable().\n",
              __FUNCTION__);
    else
      valid = true;
  }
  if (event_type & (WB_EVENT_JOYSTICK_BUTTON | WB_EVENT_JOYSTICK_AXIS | WB_EVENT_JOYSTICK_POV)) {
    if (wb_joystick_get_sampling_period() <= 0)
      fprintf(stderr,
              "Error: %s() called with an event type including a joystick, but no joystick is enabled, please enable it with "
              "wb_joystick_enable().\n",
              __FUNCTION__);
    else
      valid = true;
  }

  if (!valid)
    return WB_EVENT_NO_EVENT;

  robot_mutex_lock();
  robot.is_waiting_for_user_input_event = true;
  robot.user_input_event_type = event_type;
  robot.user_input_event_timeout = timeout;
  wb_robot_flush_unlocked(__FUNCTION__);
  while (robot.is_waiting_for_user_input_event && !robot_is_quitting())
    robot_read_data();

  if (robot.webots_exit == WEBOTS_EXIT_NOW) {
    robot_quit();
    robot_mutex_unlock();
    exit(EXIT_SUCCESS);
  }

  if (robot.webots_exit == WEBOTS_EXIT_LATER) {
    robot.webots_exit = WEBOTS_EXIT_NOW;
    robot_mutex_unlock();
    return WB_EVENT_QUIT;
  }

  robot_mutex_unlock();
  return robot.user_input_event_type;
}

void wb_robot_flush_unlocked(const char *function) {
  if (function && waiting_for_step_end) {
    fprintf(
      stderr,
      "Warning: %s(): functions with immediate requests to Webots cannot be implemented in-between wb_robot_step_begin() and "
      "wb_robot_step_end()!\n",
      function);
    return;
  }

  if (robot.webots_exit == WEBOTS_EXIT_NOW) {
    robot_quit();
    robot_mutex_unlock();
    exit(EXIT_SUCCESS);
  }
  if (robot.webots_exit == WEBOTS_EXIT_LATER)
    return;
  robot.is_immediate_message = true;
  robot_send_request(0);
  robot_read_data();
  if (robot.webots_exit == WEBOTS_EXIT_NOW)
    robot.webots_exit = WEBOTS_EXIT_LATER;
  robot.is_immediate_message = false;
}

int wb_robot_init_msvc() {
  return wb_robot_init();
}

static void wb_robot_cleanup_devices() {
  WbDeviceTag tag;
  for (tag = 1; tag < robot.n_device; tag++) {
    WbDevice *d = robot.device[tag];
    if (d->node != WB_NODE_CAMERA)
      continue;
    wb_device_cleanup(d);
    robot.device[tag] = NULL;
  }
}

static char *encode_robot_name(const char *robot_name) {
  if (!robot_name)
    return NULL;

  char *encoded_name = percent_encode(robot_name);
  int length = strlen(encoded_name);
  // the robot name is used to connect to the libController and in this process there are indirect
  // limitations such as QLocalServer only accepting strings up to 106 characters for server names,
  // for these reasons if the robot name is bigger than an arbitrary length, a hashed version is used instead
  if (length > 70) {  // note: this threshold should be the same as in WbRobot.cpp
    char hash[21];
    char *output = malloc(41);
    SHA1(hash, encoded_name, length);
    free(encoded_name);
    for (size_t i = 0; i < 20; i++)
      sprintf((output + (2 * i)), "%02x", hash[i] & 0xff);
    return output;
  }

  return encoded_name;
}

static char *compute_socket_filename(char *error_buffer) {
  char *robot_name = encode_robot_name(wbu_system_getenv("WEBOTS_ROBOT_NAME"));
  const char *WEBOTS_INSTANCE_PATH = wbu_system_webots_instance_path(true);
  char *socket_filename;
  if (robot_name && robot_name[0] && WEBOTS_INSTANCE_PATH && WEBOTS_INSTANCE_PATH[0]) {
#ifndef _WIN32
    const int length = strlen(WEBOTS_INSTANCE_PATH) + strlen(robot_name) + 15;  // "%sintern/%s/socket"
    socket_filename = malloc(length);
    snprintf(socket_filename, length, "%sipc/%s/intern", WEBOTS_INSTANCE_PATH, robot_name);
#else
    int i = 0;
    int last = -1;
    while (WEBOTS_INSTANCE_PATH[i] != 0) {
      if (WEBOTS_INSTANCE_PATH[i++] == '-')
        last = i;
    }
    int number = -1;
    sscanf(&WEBOTS_INSTANCE_PATH[last], "%d", &number);
    assert(number != -1);
    const int length = 28 + strlen(robot_name);  // "\\.\pipe\webots-XXXX-robot_name"
    socket_filename = malloc(length);
    snprintf(socket_filename, length, "\\\\.\\pipe\\webots-%d-%s", number, robot_name);
#endif
    free(robot_name);
    return socket_filename;
  }

#ifndef _WIN32
  const char *username = wbu_system_getenv("USER");
  if (username == NULL || username[0] == '\0') {
    username = wbu_system_getenv("USERNAME");
    if (username == NULL || username[0] == '\0') {
      fprintf(stderr, "Error: USER or USERNAME environment variable not set, falling back to 'default' username.\n");
      username = "default";
    }
  }
#endif

  // extern controller case
  // parse WEBOTS_CONTROLLER_URL to extract protocol, host, port and robot name
  const char *TMP_DIR = wbu_system_tmpdir();
  char *WEBOTS_CONTROLLER_URL = (char *)wbu_system_getenv("WEBOTS_CONTROLLER_URL");
  // either the WEBOTS_CONTROLLER_URL is not defined, empty or contains only a robot name
  // default to the most recent /tmp/webots/username/* folder (/tmp/webots-* on Windows)
  if (WEBOTS_CONTROLLER_URL == NULL || WEBOTS_CONTROLLER_URL[0] == 0 || strstr(WEBOTS_CONTROLLER_URL, "://") == NULL) {
#ifndef _WIN32
    const int WEBOTS_TMP_DIR_length = strlen(TMP_DIR) + strlen(username) + 9;  // TMP_DIR + '/webots/' + username
    char *WEBOTS_TMP_DIR = malloc(WEBOTS_TMP_DIR_length);
    snprintf(WEBOTS_TMP_DIR, WEBOTS_TMP_DIR_length, "%s/webots/%s", TMP_DIR, username);
#else
    const int WEBOTS_TMP_DIR_length = strlen(TMP_DIR);
    char *WEBOTS_TMP_DIR = strdup(TMP_DIR);
#endif
    DIR *dr = opendir(WEBOTS_TMP_DIR);
    if (dr == NULL) {
      snprintf(error_buffer, ERROR_BUFFER_SIZE, "Cannot open directory %s", WEBOTS_TMP_DIR);
      free(WEBOTS_TMP_DIR);
      return NULL;
    }
    struct stat filestat;
    double timestamp = 0.0;
    int number = -1;
    const struct dirent *de;
    while ((de = readdir(dr)) != NULL) {
#ifndef _WIN32
      if (strcmp(de->d_name, ".") && strcmp(de->d_name, "..")) {
#else
      if (strcmp(de->d_name, ".") && strcmp(de->d_name, "..") && !strncmp(de->d_name, "webots-", 7)) {
#endif
        const int length = WEBOTS_TMP_DIR_length + strlen(de->d_name) + 2;
        char *filename = malloc(length);
        snprintf(filename, length, "%s/%s", WEBOTS_TMP_DIR, de->d_name);
        if (stat(filename, &filestat) == 0) {
#ifdef _WIN32
          double ts = (double)filestat.st_mtime;
#else
          double ts = filestat.st_mtim.tv_sec + (filestat.st_mtim.tv_nsec / 1000000000.0);  // last modification time
#endif
          // printf("ts = %.17lg\n", ts);
          if (ts > timestamp) {
            timestamp = ts;
#ifndef _WIN32
            sscanf(de->d_name, "%d", &number);
#else
            sscanf(de->d_name, "webots-%d", &number);
#endif
          }
        }
        free(filename);
      }
    }
    closedir(dr);
    free(WEBOTS_TMP_DIR);
    // No Webots instance has been started
    if (number == -1) {
      snprintf(error_buffer, ERROR_BUFFER_SIZE, "Cannot find any instance of Webots");
      return NULL;
    }
    if (WEBOTS_CONTROLLER_URL && WEBOTS_CONTROLLER_URL[0]) {  // only the robot name was provided in WEBOTS_CONTROLLER_URL
      const int length = 19 + strlen(WEBOTS_CONTROLLER_URL);
      char *tmp = malloc(length);
      snprintf(tmp, length, "ipc://%d/%s", number, WEBOTS_CONTROLLER_URL);
      WEBOTS_CONTROLLER_URL = tmp;
    } else {
      WEBOTS_CONTROLLER_URL = malloc(18);
      snprintf(WEBOTS_CONTROLLER_URL, 18, "ipc://%d", number);
    }
  } else
    WEBOTS_CONTROLLER_URL = strdup(WEBOTS_CONTROLLER_URL);  // it will be free
  if (strncmp(WEBOTS_CONTROLLER_URL, "ipc://", 6) != 0) {
    fprintf(stderr, "Error: unsupported protocol in WEBOTS_CONTROLLER_URL: %s\n", WEBOTS_CONTROLLER_URL);
    exit(EXIT_FAILURE);
  }
  // fprintf(stderr, "WEBOTS_CONTROLLER_URL=%s\n", WEBOTS_CONTROLLER_URL);
  int number = -1;
  sscanf(&WEBOTS_CONTROLLER_URL[6], "%d", &number);
  if (number == -1) {
    fprintf(stderr, "Error: invalid WEBOTS_CONTROLLER_URL: %s (missing or wrong port value)\n", WEBOTS_CONTROLLER_URL);
    exit(EXIT_FAILURE);
  }
#ifdef _WIN32
  const int webots_instance_path_length = strlen(TMP_DIR) + 20;  // TMP_DIR + "/webots-12345678901"
  const int ipc_path_length = webots_instance_path_length + 4;   // TMP_DIR + "/webots-12345678901/ipc"
#else
  const int webots_instance_path_length =
    strlen(TMP_DIR) + strlen(username) + 21;                    // TMP_DIR + '/webots/' + username + '/12345678901'
  const int ipc_path_length = webots_instance_path_length + 4;  // TMP_DIR + '/webots/' + username + '/12345678901/ipc'
#endif
  char *webots_instance_folder = malloc(webots_instance_path_length);
  char *ipc_folder = malloc(ipc_path_length);
#ifdef _WIN32
  snprintf(webots_instance_folder, webots_instance_path_length, "%s/webots-%d", TMP_DIR, number);
  snprintf(ipc_folder, ipc_path_length, "%s/ipc", webots_instance_folder);
#else
  snprintf(webots_instance_folder, webots_instance_path_length, "%s/webots/%s/%d", TMP_DIR, username, number);
  snprintf(ipc_folder, ipc_path_length, "%s/ipc", webots_instance_folder);
#endif

  // check if the Webots instance has been started
  DIR *dr = opendir(webots_instance_folder);
  if (dr == NULL) {
    snprintf(error_buffer, ERROR_BUFFER_SIZE, "Cannot connect to Webots instance");
    free(webots_instance_folder);
    free(ipc_folder);
    return NULL;
  }
  closedir(dr);
  // check if Webots is currently loading
  char *loading_file_path = malloc(strlen(webots_instance_folder) + 9);  // webots_instance_folder + '/loading'
  snprintf(loading_file_path, strlen(webots_instance_folder) + 9, "%s/loading", webots_instance_folder);
  FILE *loading_file = fopen(loading_file_path, "r");
  if (loading_file) {
    fclose(loading_file);
    snprintf(error_buffer, ERROR_BUFFER_SIZE, "The Webots simulation world is not yet ready");
    free(webots_instance_folder);
    free(ipc_folder);
    free(loading_file_path);
    return NULL;
  }
  free(webots_instance_folder);
  free(loading_file_path);

  free(robot_name);
  const char *sub_string = strstr(&WEBOTS_CONTROLLER_URL[6], "/");
  robot_name = encode_robot_name(sub_string ? sub_string + 1 : NULL);
  if (robot_name) {
#ifndef _WIN32
    // socket file name is like: folder + robot_name + "/extern"
    const int length = ipc_path_length + strlen(robot_name) + 8;
    socket_filename = malloc(length);
    snprintf(socket_filename, length, "%s/%s/extern", ipc_folder, robot_name);
#else
    // socket file name is like: "\\.\\pipe\webots-XXX-robot_name"
    const int length = 28 + strlen(robot_name);
    socket_filename = malloc(length);
    snprintf(socket_filename, length, "\\\\.\\pipe\\webots-%d-%s", number, robot_name);
#endif
    free(robot_name);
  } else {  // check if a single extern robot is present in the ipc folder
    dr = opendir(ipc_folder);
    if (dr == NULL) {  // the ipc folder was not yet created
      free(ipc_folder);
      snprintf(error_buffer, ERROR_BUFFER_SIZE,
               "The Webots simulation has not yet started or there is no robot in the simulation");
      return NULL;
    }
    char **filenames = NULL;
    int count = 0;
    const struct dirent *de;
    while ((de = readdir(dr)) != NULL) {
      if (strcmp(de->d_name, ".") == 0 || strcmp(de->d_name, "..") == 0)
        continue;
      bool found = false;
      // search the robot folder for a file named "extern"
      const int l = ipc_path_length + strlen(de->d_name) + 1;
      char *subfolder = malloc(l);
      snprintf(subfolder, l, "%s/%s", ipc_folder, de->d_name);
      DIR *d = opendir(subfolder);
      free(subfolder);
      if (d) {
        const struct dirent *sub;
        while ((sub = readdir(d)) != NULL) {
          if (strcmp(sub->d_name, "extern") == 0) {
            found = true;
            break;
          }
        }
      }
      if (!found)
        continue;
      char **r = realloc(filenames, (count + 1) * sizeof(char *));
      if (!r) {
        fprintf(stderr, "Cannot allocate memory for listing \"%s\" folder.\n", ipc_folder);
        exit(EXIT_FAILURE);
      }
      filenames = r;
      filenames[count++] = strdup(de->d_name);
    }
    closedir(dr);
    if (count == 0) {
      snprintf(error_buffer, ERROR_BUFFER_SIZE,
               "The Webots simulation has not yet started or there is no robot with an <extern> controller");
      socket_filename = NULL;
    } else if (count > 1) {  // more than one extern controller in the current instance of Webots
      fprintf(stderr,
              "No robot name provided, Webots instance %d should have exactly one robot set with an <extern> controller.\n",
              number);
      fprintf(stderr, "Available robots with <extern> controllers are:\n");
      for (int i = 0; i < count; i++)
        fprintf(stderr, " * %s\n", filenames[i]);
      exit(EXIT_FAILURE);
    } else {
#ifndef _WIN32
      const int l = ipc_path_length + strlen(filenames[0] + 1) + 8;  // folder + robot_name + "/extern"
      socket_filename = malloc(l);
      snprintf(socket_filename, l, "%s/%s/extern", ipc_folder, filenames[0]);
#else
      const int l = 28 + strlen(filenames[0] + 1);  // "\\.\pipe\webots-XXXX" + robot_name
      socket_filename = malloc(l);
      snprintf(socket_filename, l, "\\\\.\\pipe\\webots-%d-%s", number, filenames[0]);
#endif
    }
    for (int i = 0; i < count; i++)
      free(filenames[i]);
    free(filenames);
  }
  free(ipc_folder);
  free(WEBOTS_CONTROLLER_URL);
  return socket_filename;
}

static void compute_remote_info(char **host, int *port, char **robot_name) {
  const char *WEBOTS_CONTROLLER_URL = wbu_system_getenv("WEBOTS_CONTROLLER_URL");
  const char *url_suffix = strstr(&WEBOTS_CONTROLLER_URL[6], ":");

  if (url_suffix == NULL) {  // assuming only the IP address was provided
    fprintf(stderr, "Error: Missing port in WEBOTS_CONTROLLER_URL: %s\n", WEBOTS_CONTROLLER_URL);
    exit(EXIT_FAILURE);
  } else if (WEBOTS_CONTROLLER_URL[6] == ':') {
    fprintf(stderr, "Error: Missing IP address in WEBOTS_CONTROLLER_URL: %s\n", WEBOTS_CONTROLLER_URL);
    exit(EXIT_FAILURE);
  }
  const int host_length = strlen(&WEBOTS_CONTROLLER_URL[6]) - strlen(url_suffix) + 1;
  *host = malloc(host_length);
  snprintf(*host, host_length, "%s", &WEBOTS_CONTROLLER_URL[6]);
  sscanf(url_suffix, ":%d", port);
  const char *rn = strstr(url_suffix, "/");
  *robot_name = rn != NULL ? encode_robot_name(rn + 1) : NULL;
}

int wb_robot_init() {  // API initialization
// do not use any buffer for the standard streams
#ifdef _WIN32  // the line buffered option doesn't to work under Windows, so use unbuffered streams
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);
#else
  setvbuf(stdout, NULL, _IOLBF, 4096);
  setvbuf(stderr, NULL, _IOLBF, 4096);
#endif
  // flush stdout and stderr to display messages printed before the robot initialization in the Webots console
  fflush(stdout);
  fflush(stderr);
  static bool already_done = false;
  if (already_done)
    return true;
  setlocale(LC_NUMERIC, "C");
  // the robot.configuration points to a data structure is made up of the following:
  // one uint8 saying if the robot is synchronized (1) or not (0)
  // one uint8 giving the number of devices n
  // n \0-terminated strings giving the names of the devices 0 .. n-1

  signal(SIGINT, quit_controller);  // this signal is working on Windows when Ctrl+C from cmd.exe.
#ifndef _WIN32
  signal(SIGTERM, quit_controller);
  signal(SIGQUIT, quit_controller);
  signal(SIGHUP, quit_controller);
#endif

  robot.configure = 0;
  robot.real_robot_cleanup = NULL;
  robot.is_supervisor = false;
  robot.synchronization = 0;
  robot.mode = 0;  // simulation
  robot.client_exit = false;
  robot.webots_exit = WEBOTS_EXIT_FALSE;
  robot.battery_value = NAN;
  robot.battery_sampling_period = 0;  // initially disabled
  robot.console_stdout = NULL;
  robot.console_stderr = NULL;
  robot.urdf = NULL;
  robot.urdf_prefix = NULL;
  robot.need_urdf = false;
  robot.pin = -1;
  robot.is_waiting_for_user_input_event = false;
  robot.dataNeedToWriteRequest = false;
  wb_keyboard_init();
  wb_joystick_init();
  wb_mouse_init();

  int retry = 0;
  while (true) {
    bool success;
    bool is_simulation_loading;
    const char *WEBOTS_CONTROLLER_URL = wbu_system_getenv("WEBOTS_CONTROLLER_URL");
    const char *WEBOTS_ROBOT_NAME = wbu_system_getenv("WEBOTS_ROBOT_NAME");
    const char *WEBOTS_TMP_PATH = wbu_system_webots_instance_path(true);
    char *error_message = malloc(ERROR_BUFFER_SIZE);
    char *socket_filename = NULL;
    if ((WEBOTS_CONTROLLER_URL != NULL) &&
        !(WEBOTS_ROBOT_NAME && WEBOTS_ROBOT_NAME[0] && WEBOTS_TMP_PATH && WEBOTS_TMP_PATH[0]) &&
        strncmp(WEBOTS_CONTROLLER_URL, "tcp://", 6) == 0) {  // TCP URL given and not an intern controller
      char *host, *robot_name;
      int port = -1;
      compute_remote_info(&host, &port, &robot_name);
      success = scheduler_init_remote(host, port, robot_name, error_message);
      is_simulation_loading = strncmp(error_message, "The Webots simulation world is not yet ready", 44) == 0;
      if (success) {
        free(host);
        free(robot_name);
        free(error_message);
        break;
      }
      free(host);
      free(robot_name);
    } else {  // Intern or IPC extern controller
      socket_filename = compute_socket_filename(error_message);
      success = socket_filename ? scheduler_init_local(socket_filename) : false;
      is_simulation_loading = strncmp(error_message, "The Webots simulation world is not yet ready", 44) == 0;
      if (success) {
        free(socket_filename);
        free(error_message);
        break;
      }
    }
    if (retry % 5 == 0 && retry != 50) {
      if (is_simulation_loading) {
        retry -= 5;
        fprintf(stderr, "%s, pending until loading is done...\n", error_message);
      } else if (socket_filename) {
        free(socket_filename);
        fprintf(
          stderr,
          "The specified robot is not in the list of robots with <extern> controllers, retrying for another %d seconds...\n",
          50 - retry);
      } else
        fprintf(stderr, "%s, retrying for another %d seconds...\n", error_message, 50 - retry);
    }
    free(error_message);
    if (retry++ > 50) {
      fprintf(stderr, "Giving up...\n");
      exit(EXIT_FAILURE);
    }
    sleep(1);
  }
  if (getenv("WEBOTS_STDOUT_REDIRECT"))
    stdout_read = stream_pipe_create(1);
  if (getenv("WEBOTS_STDERR_REDIRECT"))
    stderr_read = stream_pipe_create(2);

  // robot device
  robot.n_device = 1;
  robot.device = malloc(sizeof(WbDevice *));
  robot.device[0] = malloc(sizeof(WbDevice));
  robot.device[0]->name = NULL;
  robot.device[0]->node = WB_NODE_ROBOT;
  robot.device[0]->read_answer = robot_read_answer;
  robot.device[0]->write_request = robot_write_request;
  robot.device[0]->cleanup = NULL;
  robot_init_was_done = true;
  robot_step_mutex = wb_robot_mutex_new();
  robot.device[0]->toggle_remote = robot_toggle_remote;
  robot.toggle_remote_first_step = false;
  robot.send_remote_mode_off = false;
  robot.show_window = false;
  robot.update_window = false;
  robot.has_html_robot_window = false;
  robot.wwi_message_to_send = NULL;
  robot.wwi_message_to_send_size = 0;
  robot.wwi_reception_buffer = NULL;
  robot.wwi_received_messages_size = 0;
  robot.wwi_reception_buffer_size = 0;
  robot.wwi_reset_reading_head = true;
  robot.simulation_mode = -1;

  // receive a configure message for the robot and devices
  // need to check return value to avoid crash when the
  // user reverts before having run the simulation
  if (wb_robot_step(0) == -1) {
    robot_quit();
    exit(EXIT_SUCCESS);
  }

  init_remote_control_library();

  // show the robot window as early as possible if required
  if (robot.show_window && !robot.has_html_robot_window) {
    init_robot_window_library();
    robot_window_show();
    robot.show_window = false;
  }

  robot.configure = 0;

  atexit(wb_robot_cleanup_devices);

  already_done = true;

  html_robot_window_init();
  return true;
}

WbRobotMode wb_robot_get_mode() {
  return robot.mode;
}

double wb_robot_get_basic_time_step() {
  return robot.basic_time_step;
}

double wb_robot_get_time() {
  return simulation_time;
}

bool wb_robot_get_synchronization() {
  return robot.synchronization;
}

bool wb_robot_get_supervisor() {
  return robot.is_supervisor;
}

int wb_robot_get_step_duration() {
  return current_step_duration;
}

const char *wb_robot_get_project_path() {
  return robot.project_path;
}

const char *wb_robot_get_world_path() {
  return robot.world_path;
}

const char *wb_robot_get_controller_name() {
  return robot.controller_name;
}

void wb_robot_pin_to_static_environment(bool pin) {
  robot.pin = pin ? 1 : 0;
}

void wb_robot_wwi_send(const char *data, int size) {
  robot_mutex_lock();
  robot.wwi_message_to_send_size = size;
  robot.wwi_message_to_send = data;
  wb_robot_flush_unlocked(__FUNCTION__);
  robot_mutex_unlock();
}

const char *wb_robot_wwi_receive(int *size) {
  if (robot.wwi_received_messages_size) {
    if (size)
      *size = robot.wwi_received_messages_size;
    robot.wwi_received_messages_size = 0;
    return robot.wwi_reception_buffer;
  } else {
    if (size)
      *size = 0;
    return NULL;
  }
}

const char *wb_robot_wwi_receive_text() {
  static int length;
  static int character_read = 0;
  static const char *message;
  if (robot.wwi_reset_reading_head) {
    robot.wwi_reset_reading_head = false;
    character_read = 0;
    message = wb_robot_wwi_receive(&length);
  }

  if (character_read < length && message) {
    const int current_message_length = strlen(message) + 1;
    character_read += current_message_length;
    const char *current_message = message;
    message += current_message_length;
    return current_message;
  } else
    return NULL;
}

WbSimulationMode robot_get_simulation_mode() {
  return robot.simulation_mode;
}

void robot_set_simulation_mode(WbSimulationMode mode) {
  robot.simulation_mode = mode;
}

const char *wb_robot_get_urdf(const char *prefix) {
  robot_mutex_lock();

  robot.need_urdf = true;
  free(robot.urdf_prefix);
  robot.urdf_prefix = malloc(strlen(prefix) + 1);
  strcpy(robot.urdf_prefix, prefix);

  wb_robot_flush_unlocked(__FUNCTION__);
  robot.need_urdf = false;

  robot_mutex_unlock();
  return robot.urdf;
}
