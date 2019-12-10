#include <stdio.h>
#include <webots/remote_control.h>
#include <webots/types.h>

static void logMsg(const char *msg) {
  FILE *f = fopen("log.txt", "a");
  fprintf(f, "%s\n", msg);
  fclose(f);
}

static bool wbr_start(const char *args) {
  char buffer[128];
  sprintf(buffer, "wbr_start %s", args);
  logMsg(buffer);
  return true;
}

static void wbr_stop() {
  logMsg("wbr_stop");
}

static bool wbr_has_failed() {
  logMsg("wbr_has_failed");
  return false;
}

static int wbr_robot_step(int duration) {
  logMsg("wbr_robot_step");
  return 0;
}

static void wbr_stop_actuators() {
  logMsg("wbr_stop_actuators");
}

static void wbr_set_sampling_period(WbDeviceTag tag, int sampling_period) {
  char buffer[128];
  sprintf(buffer, "wbr_set_sampling_period %d", sampling_period);
  logMsg(buffer);
}

static void wbr_led_set(WbDeviceTag tag, int state) {
  char buffer[128];
  sprintf(buffer, "wbr_led_set %d", state);
  logMsg(buffer);
}

bool wbr_init(WbrInterface *ri) {
  logMsg("wbr_init");

  ri->mandatory.wbr_start = wbr_start;
  ri->mandatory.wbr_stop = wbr_stop;
  ri->mandatory.wbr_has_failed = wbr_has_failed;
  ri->mandatory.wbr_robot_step = wbr_robot_step;
  ri->mandatory.wbr_stop_actuators = wbr_stop_actuators;

  ri->wbr_set_sampling_period = wbr_set_sampling_period;
  ri->wbr_led_set = wbr_led_set;

  return true;
}

void wbr_cleanup() {
  logMsg("wbr_cleanup");
}
