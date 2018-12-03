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

  const double *values = wb_accelerometer_get_values(acc);

  const double expected[] = {0.0, 9.81, 0.0};

  int i;
  for (i = 0; i < 3; i++)
    ts_assert_double_in_delta(values[i], expected[i], 0.001, "The Accelerometer doesn't return the right acceleration.");

  ts_send_success();
  return EXIT_SUCCESS;
}
