#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define FILE_NAME_PNG "image.png"
#define FILE_NAME_JPG "image.jpg"

bool file_exists(const char *filename) {
  FILE *file = fopen(filename, "r");
  if (file) {
    fclose(file);
    return true;
  }
  return false;
}

int main(int argc, char **argv) {
  int i;

  ts_setup(argv[0]);

  int time_step = wb_robot_get_basic_time_step();

  remove(FILE_NAME_PNG);
  remove(FILE_NAME_JPG);

  ts_assert_boolean_not_equal(file_exists(FILE_NAME_PNG), "Impossible to remove the PNG file");
  ts_assert_boolean_not_equal(file_exists(FILE_NAME_JPG), "Impossible to remove the JPG file");

  wb_robot_step(time_step);

  wb_supervisor_export_image(FILE_NAME_PNG, 100);
  wb_supervisor_export_image(FILE_NAME_JPG, 100);

  wb_robot_step(time_step);

  // We need to allow some time to check the existence of image files
  // as the test sometimes fails on Windows, probably because the system
  // needs some time to make the file available to other applications
  // after it was just created.
  for (i = 0; i < 10; i++) {
    if (file_exists(FILE_NAME_PNG) && file_exists(FILE_NAME_JPG))
      break;
    wb_robot_step(time_step);
  }

  ts_assert_boolean_equal(file_exists(FILE_NAME_PNG),
                          "wb_supervisor_export_image() failed to create the " FILE_NAME_PNG " file.");

  ts_assert_boolean_equal(file_exists(FILE_NAME_JPG),
                          "wb_supervisor_export_image() failed to create the " FILE_NAME_JPG " file.");

  ts_send_success();
  return EXIT_SUCCESS;
}
