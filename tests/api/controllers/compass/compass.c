#include <webots/compass.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag compass = wb_robot_get_device("compass");

  wb_compass_enable(compass, TIME_STEP);

  wb_robot_step(TIME_STEP);

  const double *values = wb_compass_get_values(compass);

  const double expected[] = {-0.707107, 0.000000, -0.707107};

  int i;
  for (i = 0; i < 3; i++)
    ts_assert_double_in_delta(values[i], expected[i], 0.0001, "The compass doesn't return the right north direction.");

  ts_send_success();
  return EXIT_SUCCESS;
}
