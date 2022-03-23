#include <webots/range_finder.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <string.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_assert_boolean_equal(argc == 2, "range_finder_checker badly used. world name expected as argument.");

  ts_setup(argv[1]);

  WbDeviceTag range_finder = wb_robot_get_device("range-finder");
  ts_assert_boolean_equal(range_finder, "RangeFinder device cannot be get.");

  int width = wb_range_finder_get_width(range_finder);
  int height = wb_range_finder_get_height(range_finder);

  ts_assert_boolean_equal(width >= 1 && height >= 1, "Unexpected dimension: %dx%d", width, height);

  wb_range_finder_enable(range_finder, TIME_STEP);

  wb_robot_step(TIME_STEP);

  if (strcmp(argv[1], "range_finder") == 0) {
    float v;
    int x, y;
    const float *image = wb_range_finder_get_range_image(range_finder);

    const int samples_number = 5;
    const int samples_positions[5][2] = {
      {width / 2, 0},           // top line, central pixel
      {0, height / 2},          // middle line, first pixel
      {width / 2, height / 2},  // middle line, central pixel
      {width - 1, height / 2},  // middle line, last pixel
      {width / 2, height - 1}   // bottom line, central pixel
    };
    const double samples_expected_values[5] = {
      INFINITY,  // no collision
      INFINITY,  // no collision
      2.0,       // collision with transparent object
      2.0,       // collision
      2.0        // collision with colored ElevationGrid
    };

    int i;
    for (i = 0; i < samples_number; i++) {
      x = samples_positions[i][0];
      y = samples_positions[i][1];
      v = wb_range_finder_image_get_depth(image, width, x, y);
      ts_assert_double_in_delta(v, samples_expected_values[i], 0.015,
                                "Wrong value at (%d, %d), Received value = %f, Expected value = %f", x, y, v,
                                samples_expected_values[i]);
    }
  } else if (strcmp(argv[1], "range_finder_spherical") == 0) {
    float v;
    int x, y;
    const float *image = wb_range_finder_get_range_image(range_finder);

    const int samples_number = 5;
    const int samples_positions[5][2] = {
      {0, height / 2},              // middle line, first pixel
      {width / 4, height / 2},      // middle line, fourth
      {width / 2, height / 2},      // middle line, middle
      {3 * width / 4, height / 2},  // middle line, 3 fourth
      {width - 1, height / 2},      // middle line, last pixel
    };
    const double samples_expected_values[5] = {
      2.0,       // collision
      INFINITY,  // no collision
      2.0,       // collision
      INFINITY,  // no collision
      2.0,       // collision
    };

    int i;
    for (i = 0; i < samples_number; i++) {
      x = samples_positions[i][0];
      y = samples_positions[i][1];
      v = wb_range_finder_image_get_depth(image, width, x, y);
      ts_assert_double_in_delta(v, samples_expected_values[i], 0.015,
                                "Wrong value at (%d, %d), Received value = %f, Expected value = %f", x, y, v,
                                samples_expected_values[i]);
    }
  } else
    ts_send_error_and_exit("range_finder_checker doesn't support this world: %s", argv[1]);

  ts_assert_boolean_equal(wb_range_finder_save_image(range_finder, "test.hdr", 100) != -1,
                          "range_finder image did not save correctly (wb_range_finder_save_image returned -1)");

  ts_send_success();
  return EXIT_SUCCESS;
}
