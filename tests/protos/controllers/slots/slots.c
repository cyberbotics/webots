#include <webots/gps.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // get and enable the gps
  WbDeviceTag gps = wb_robot_get_device("mygps");
  wb_gps_enable(gps, TIME_STEP);

  // wait 2 second
  int i;
  for (i = 0; i < 2000 / TIME_STEP; ++i)
    wb_robot_step(TIME_STEP);

  const double *values = wb_gps_get_values(gps);
  const double y_value = values[1];

  // check the gps value is correct
  // if physics doesn't work, either the GPS will be blocked above, or will go through the plane
  ts_assert_boolean_equal(!isnan(y_value), "GPS value is NaN");
  printf("y-value = %f\n", y_value);
  const double tolerance = 0.01;
  ts_assert_boolean_equal(y_value < 0.05 + tolerance, "GPS is above the threshold. Physics issue detected");
  ts_assert_boolean_equal(y_value > 0.05 - tolerance, "GPS is below the threshold. Physics issue detected");

  ts_send_success();
  return EXIT_SUCCESS;
}
