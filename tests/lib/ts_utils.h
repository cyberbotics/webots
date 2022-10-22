#ifndef TSE_UTILS_H
#define TSE_UTILS_H

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/types.h>
#include <unistd.h>
#endif

#include <webots/emitter.h>
#include <webots/robot.h>

static const char *ts_results_filename = "../../../output.txt";
static char ts_test_name[256];
static bool ts_setup_done = false;
static bool ts_disable_output_log_flag = false;
static WbDeviceTag ts_emitter = 0;

static void ts_notify_controller_status(bool running) {
  if (ts_emitter == 0)
    ts_emitter = wb_robot_get_device("ts_emitter");

  char msg[256];
#ifdef _WIN32
  sprintf(msg, "ts %d %ld", running, GetCurrentProcessId());
#else
  sprintf(msg, "ts %d %d", running, getpid());
#endif
  wb_emitter_send(ts_emitter, msg, strlen(msg));
}

void ts_set_test_name(const char *test_name) {
  int len = (int)strlen(test_name);
  int i;

  // remove the full path

  for (i = len - 1; i >= 0; i--) {
    if (test_name[i] == '/' || test_name[i] == '\\')
      break;
  }

  strcpy(ts_test_name, &test_name[i + 1]);

  // keep the basename (remove the .exe extension)
  for (i = 0; i < 256; i++) {
    if (ts_test_name[i] == '.')
      ts_test_name[i] = '\0';
  }
}

void ts_setup(const char *test_name) {
  wb_robot_init();

  ts_set_test_name(test_name);
  ts_setup_done = true;
  ts_notify_controller_status(true);
}

void ts_disable_output_log() {
  ts_disable_output_log_flag = true;
}

void ts_send_success() {
  assert(ts_setup_done);

  // write the result to the file
  if (!ts_disable_output_log_flag) {
    FILE *f = fopen(ts_results_filename, "a");
    fprintf(f, "OK: %s\n", ts_test_name);
    fclose(f);
  }

  // write the result to the console
  printf("OK: %s\n", ts_test_name);
  ts_notify_controller_status(false);

  wb_robot_cleanup();
  exit(EXIT_SUCCESS);
}

bool ts_is_pointer_size_64_bits() {
  return (sizeof(void *) == 8);
}

// please use rather the assertion tests that this function
void ts_v_send_error_and_exit(const char *error_format, va_list args) {
  assert(ts_setup_done);

  // copy va_list to use it twice
  va_list copied_args;
  va_copy(copied_args, args);

  // write the result to the file
  if (!ts_disable_output_log_flag) {
    FILE *f = fopen(ts_results_filename, "a");
    fprintf(f, "FAILURE with %s: ", ts_test_name);
    vfprintf(f, error_format, args);
    fprintf(f, "\n");
    fclose(f);
  }

  // write the result to the console
  printf("FAILURE with %s: ", ts_test_name);
  vprintf(error_format, copied_args);
  printf("\n");

  // cleanup
  va_end(copied_args);
  ts_notify_controller_status(false);
  wb_robot_cleanup();
  exit(EXIT_FAILURE);
}

// please use rather the assertion tests that this function
void ts_send_error_and_exit(const char *error_format, ...) {
  assert(ts_setup_done);
  va_list args;

  va_start(args, error_format);
  // write the result to the file
  if (!ts_disable_output_log_flag) {
    FILE *f = fopen(ts_results_filename, "a");
    fprintf(f, "FAILURE with %s: ", ts_test_name);
    vfprintf(f, error_format, args);
    fprintf(f, "\n");
    fclose(f);
  }

  // write the result to the console
  printf("FAILURE with %s: ", ts_test_name);
  vprintf(error_format, args);
  printf("\n");

  // cleanup
  va_end(args);
  ts_notify_controller_status(false);
  wb_robot_cleanup();
  exit(EXIT_FAILURE);
}

#endif
