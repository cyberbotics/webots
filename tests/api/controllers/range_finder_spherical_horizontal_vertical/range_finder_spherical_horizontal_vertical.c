#include <webots/range_finder.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <string.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag horizontal_range_finder = wb_robot_get_device("horizontal_range_finder");
  WbDeviceTag vertical_range_finder = wb_robot_get_device("vertical_range_finder");

  int width = wb_range_finder_get_width(horizontal_range_finder);
  int height = wb_range_finder_get_height(vertical_range_finder);
  ts_assert_int_equal(width, height,
                      "The width of the horizontal range-finder should be equal to the height of the vertical range-finder.");

  wb_range_finder_enable(horizontal_range_finder, TIME_STEP);
  wb_range_finder_enable(vertical_range_finder, TIME_STEP);

  wb_robot_step(TIME_STEP);

  const float *image_horizontal = wb_range_finder_get_range_image(horizontal_range_finder);
  const float *image_vertical = wb_range_finder_get_range_image(vertical_range_finder);
  int i = 0;
  for (i = 0; i < width; ++i) {
    fprintf(stdout, "H= %lf (0,%d), V= %lf (%d,0).\n", image_horizontal[i], i, image_vertical[i], i);
  }
  ts_assert_int_equal(0, 1, "Test to see values on CI");
  ts_send_success();
  return EXIT_SUCCESS;
}
