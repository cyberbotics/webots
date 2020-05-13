#include <webots/accelerometer.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag acc = wb_robot_get_device("accelerometer");

  wb_accelerometer_enable(acc, TIME_STEP);

  wb_robot_step(TIME_STEP);

  int lookup_table_size = wb_accelerometer_get_lookup_table_size(acc);
  ts_assert_double_equal(lookup_table_size, 2, "Lookup table size returned is wrong (%d instead of 2)", lookup_table_size);
  const double *lookup_table = wb_accelerometer_get_lookup_table(acc);
  ts_assert_double_equal(lookup_table[3], 1000, "Lookup table (index 3) returned is wrong (%lf instead of 1000)",
                         lookup_table[3]);
  ts_assert_double_equal(lookup_table[5], 0, "Lookup table (index 5) returned is wrong (%lf instead of 0)", lookup_table[5]);

  const double *values = wb_accelerometer_get_values(acc);

  const double expected[] = {0.0, 9.81, 0.0};

  int i;
  for (i = 0; i < 3; i++)
    ts_assert_double_in_delta(values[i], expected[i], 0.001, "The Accelerometer doesn't return the right acceleration.");

  ts_send_success();
  return EXIT_SUCCESS;
}
