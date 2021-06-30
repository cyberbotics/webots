#include <webots/altimeter.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag altimeter;
  int j;
  double h;
  const double e = 0.05;

  altimeter = wb_robot_get_device("altimeter");

  h = wb_altimeter_get_value(altimeter);
  ts_assert_double_equal(h, NAN, "The altitude measured by the Altimeter should be NAN and not %g before the device is enabled",
                         h);

  wb_altimeter_enable(altimeter, TIME_STEP);

  h = wb_altimeter_get_value(altimeter);

  ts_assert_double_equal(
    h, NAN, "The altitude measured by the Altimeter should be NAN and not %g before a wb_robot_step is performed", h);

  for (j = 1; j <= 10; j++) {
    wb_robot_step(TIME_STEP);
    h = wb_altimeter_get_value(altimeter);
    ts_assert_double_equal(h, e, "The altitude measured by the Altimeter should be %g and not %g after %d wb_robot_step(s)", e,
                           h, j);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
