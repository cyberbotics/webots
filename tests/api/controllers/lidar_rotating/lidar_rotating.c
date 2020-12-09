#include <webots/lidar.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <math.h>
#include <string.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag fixed_lidar = wb_robot_get_device("lidar");
  WbDeviceTag rotating_lidar = wb_robot_get_device("rotating lidar");

  const int fixed_lidar_resolution = wb_lidar_get_horizontal_resolution(fixed_lidar);
  const int rotating_lidar_resolution = wb_lidar_get_horizontal_resolution(rotating_lidar);

  ts_assert_int_equal(4 * fixed_lidar_resolution, rotating_lidar_resolution,
                      "The resolution of the rotating lidar should be 4 times bigger than the resolution of the fixed one.");

  wb_lidar_enable(fixed_lidar, TIME_STEP);
  wb_lidar_enable(rotating_lidar, TIME_STEP);

  // run a few steps to be sure full image of the rotating lidar has been aquired (at least one complete tour)
  int i = 0;
  for (i = 0; i < 50; ++i)
    wb_robot_step(TIME_STEP);

  // verify that part of the two layers that overlap corresponds (sum of the difference is smaller than threshold)
  const float *image_fixed_lidar = wb_lidar_get_range_image(fixed_lidar);
  ts_assert_pointer_not_null((void *)image_fixed_lidar, "Cannot retrieve range image pointer of the 'fixed' lidar.");
  const float *image_rotating_lidar = wb_lidar_get_range_image(rotating_lidar);
  ts_assert_pointer_not_null((void *)image_rotating_lidar, "Cannot retrieve range image pointer of the 'rotating' lidar.");
  double diff_sum = 0.0;

  for (i = 0; i < fixed_lidar_resolution; ++i)
    if (!isinf(image_rotating_lidar[i]) && !isinf(image_fixed_lidar[i]))
      diff_sum += fabs(image_rotating_lidar[i] - image_fixed_lidar[i]);

  ts_assert_double_is_bigger(15, diff_sum,
                             "There is too much difference in the image resulting from the fixed and from the rotating lidar, "
                             "overlapping part should be very similar");

  ts_send_success();
  return EXIT_SUCCESS;
}
