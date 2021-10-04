#include <webots/gps.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  int i;
  const double *r;
  const double e[3] = {0.0, -5.0, 0.0};
  WbDeviceTag gps = wb_robot_get_device("gps");

  double speed = wb_gps_get_speed(gps);
  ts_assert_double_equal(speed, NAN,
                         "The speed value measured by the GPS should be NaN and not %lf before the device is enabled", speed);

  r = wb_gps_get_speed_vector(gps);
  for (i = 0; i < 3; i++)
    // clang-format off
    // clang-format 11.0.0 is not compatible with previous versions with respect to nested conditional operators
    ts_assert_double_equal(r[i], NAN, "The %c value measured by the GPS should be NaN and not %g before the device is enabled",
                           i == 0 ? 'X' :
                           i == 1 ? 'Y' :
                                    'Z',
                           r[i]);
  // clang-format on

  wb_gps_enable(gps, TIME_STEP);

  speed = wb_gps_get_speed(gps);
  ts_assert_double_equal(
    speed, NAN, "The speed value measured by the GPS should be NaN and not %lf before a wb_robot_step is performed", speed);

  r = wb_gps_get_speed_vector(gps);

  for (i = 0; i < 3; i++)
    // clang-format off
    // clang-format 11.0.0 is not compatible with previous versions with respect to nested conditional operators
    ts_assert_double_equal(r[i], NAN,
                           "The %c value measured by the GPS should be NaN and not %g before a wb_robot_step is performed",
                           i == 0 ? 'X' :
                           i == 1 ? 'Y' :
                                    'Z',
                           r[i]);
  // clang-format on

  wb_robot_step(2 * TIME_STEP);

  speed = wb_gps_get_speed(gps);
  ts_assert_double_in_delta(speed, 5.0, 0.0001, "The speed value measured by the GPS should be 5.0 and not %lf after two steps",
                            speed);

  r = wb_gps_get_speed_vector(gps);
  for (i = 0; i < 3; i++)
    // clang-format off
    // clang-format 11.0.0 is not compatible with previous versions with respect to nested conditional operators
    ts_assert_double_in_delta(r[i], e[i], 0.000001,
                              "The %c value measured by the GPS should be %g and not %g after %d wb_robot_step(s)",
                              i == 0 ? 'X' :
                              i == 1 ? 'Y' :
                                       'Z',
                              e[i], r[i], 2);
  // clang-format on

  ts_send_success();
  return EXIT_SUCCESS;
}
