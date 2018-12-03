#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/remote_control.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

static const char *filename = "log.txt";

bool file_exists(const char *filename) {
  FILE *file = fopen(filename, "r");
  if (file) {
    fclose(file);
    return true;
  }
  return false;
}

int main(int argc, char **argv) {
  // remove the output file before ts_setup,
  // because the robot window will be initialized
  // during the first in-pipe configuration packet
  if (file_exists(filename))
    remove(filename);

  ts_setup(argv[0]);

  wb_robot_step(TIME_STEP);

  wb_robot_set_mode(WB_MODE_REMOTE_CONTROL, "customArgs");

  wb_robot_step(TIME_STEP);

  WbDeviceTag led = wb_robot_get_device("led");
  WbDeviceTag ds = wb_robot_get_device("distance sensor");
  wb_led_set(led, 1);
  wb_distance_sensor_enable(ds, TIME_STEP);

  double expected = 666.0;
  wbr_distance_sensor_set_value(ds, expected);
  double value = wb_distance_sensor_get_value(ds);

  ts_assert_double_equal(expected, value, "Cannot set distance sensor value (expected: %f, received: %f)\n", expected, value);

  wb_robot_step(TIME_STEP);

  FILE *file = fopen(filename, "r");
  ts_assert_pointer_not_null(file, "Remote control log file does not exist");

  wb_robot_set_mode(WB_MODE_SIMULATION, NULL);

  const char *expected_lines[] = {
    "wbr_init",       "wbr_start customArgs", "wbr_robot_step", "wbr_has_failed", "wbr_led_set 1", "wbr_set_sampling_period 32",
    "wbr_robot_step", "wbr_has_failed",       "wbr_stop",       "wbr_cleanup"};

  // read the file line by line
  int lineId = 0;
  char line[128];
  while (fgets(line, sizeof(line), file) != NULL) {
    line[strlen(line) - 1] = 0;  // remove the trailing \n character
    ts_assert_string_equal(line, expected_lines[lineId], "Error at line %d in file \"%s\": expected: \"%s\" received \"%s\"",
                           lineId, filename, expected_lines[lineId], line);
    lineId++;
  }
  fclose(file);

  ts_send_success();
  return EXIT_SUCCESS;
}
