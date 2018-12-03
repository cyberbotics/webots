#include <webots/robot.h>
#include <webots/robot_window.h>

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

  FILE *file = fopen(filename, "r");
  ts_assert_pointer_not_null(file, "Robot window log file does not exist");

  const char *expected_lines[] = {"wbw_init",       "wbw_show",         "wbw_write_actuators", "wbw_pre_update_gui",
                                  "wbw_update_gui", "wbw_read_sensors", "wbw_cleanup"};

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
