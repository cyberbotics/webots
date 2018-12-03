#include <webots/lidar.h>
#include <webots/range_finder.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <string.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag range_finder = wb_robot_get_device("range-finder");
  WbDeviceTag lidar = wb_robot_get_device("lidar");

  int range_finder_width = wb_range_finder_get_width(range_finder);
  int range_finder_height = wb_range_finder_get_height(range_finder);
  int lidar_width = wb_lidar_get_horizontal_resolution(lidar);
  int lidar_height = wb_lidar_get_number_of_layers(lidar);

  ts_assert_int_equal(range_finder_width, lidar_width,
                      "The width of the range-finder should be equal to the horizontal resolution of the lidar.");
  ts_assert_int_equal(range_finder_height, lidar_height,
                      "The height of the range-finder should be equal to the number of layers of the lidar.");

  wb_range_finder_enable(range_finder, TIME_STEP);
  wb_lidar_enable(lidar, TIME_STEP);

  wb_robot_step(TIME_STEP);

  const float *image_range_finder = wb_range_finder_get_range_image(range_finder);
  ts_assert_pointer_not_null((void *)image_range_finder, "Cannot retrieve range image pointer of the RangeFinder.");
  const float *image_lidar = wb_lidar_get_range_image(lidar);
  ts_assert_pointer_not_null((void *)image_lidar, "Cannot retrieve range image pointer of the Lidar.");
  int i = 0;
  for (i = 0; i < lidar_width; ++i)
    ts_assert_double_in_delta(image_range_finder[i], image_lidar[i], 0.05,
                              "Range-finder and lidar do not return the same value (%lf (0,%d) != %lf (%d,0)).",
                              image_range_finder[i], i, image_lidar[i], i);

  ts_send_success();
  return EXIT_SUCCESS;
}
