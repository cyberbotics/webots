#include <webots/gps.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag gps = wb_robot_get_device("gps");

  double speed = wb_gps_get_speed(gps);
  ts_assert_double_equal(speed, NAN,
                         "The speed value measured by the GPS should be NaN and not %lf before the device is enabled", speed);

  wb_gps_enable(gps, TIME_STEP);

  speed = wb_gps_get_speed(gps);
  ts_assert_double_equal(
    speed, NAN, "The speed value measured by the GPS should be NaN and not %lf before a wb_robot_step is performed", speed);

  wb_robot_step(2 * TIME_STEP);

  speed = wb_gps_get_speed(gps);
  ts_assert_double_in_delta(speed, 5.0, 0.0001, "The speed value measured by the GPS should be 5.0 and not %lf after two steps",
                            speed);

  ts_send_success();
  return EXIT_SUCCESS;
}
