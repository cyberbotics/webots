#include <stdio.h>
#include <webots/types.h>

static void logMsg(const char *msg) {
  FILE *f = fopen("log.txt", "a");
  fprintf(f, "%s\n", msg);
  fclose(f);
}

bool wbw_init() {
  logMsg("wbw_init");
  return true;
}

void wbw_cleanup() {
  logMsg("wbw_cleanup");
}

void wbw_pre_update_gui() {
  logMsg("wbw_pre_update_gui");
}

void wbw_update_gui() {
  logMsg("wbw_update_gui");
}

void wbw_read_sensors() {
  logMsg("wbw_read_sensors");
}

void wbw_write_actuators() {
  logMsg("wbw_write_actuators");
}

void wbw_show() {
  logMsg("wbw_show");
}
