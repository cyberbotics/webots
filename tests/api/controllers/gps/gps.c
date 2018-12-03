#include <webots/gps.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag gps;
  int i, j;
  const double *r;
  const double e[3] = {0.19996, 0.05, -0.35};

  gps = wb_robot_get_device("gps");

  r = wb_gps_get_values(gps);
  for (i = 0; i < 3; i++)
    ts_assert_double_equal(r[i], NAN, "The %c value measured by the GPS should be NaN and not %g before the device is enabled",
                           i == 0 ? 'X' : i == 1 ? 'Y' : 'Z', r[i]);

  wb_gps_enable(gps, TIME_STEP);

  r = wb_gps_get_values(gps);

  for (i = 0; i < 3; i++)
    ts_assert_double_equal(r[i], NAN,
                           "The %c value measured by the GPS should be NaN and not %g before a wb_robot_step is performed",
                           i == 0 ? 'X' : i == 1 ? 'Y' : 'Z', r[i]);

  int coordinate_system = wb_gps_get_coordinate_system(gps);
  ts_assert_int_equal(coordinate_system, WB_GPS_LOCAL_COORDINATE, "Wrong coordinate system returned");

  for (j = 1; j <= 10; j++) {
    wb_robot_step(TIME_STEP);
    r = wb_gps_get_values(gps);
    for (i = 0; i < 3; i++)
      ts_assert_double_in_delta(r[i], e[i], 0.000001,
                                "The %c value measured by the GPS should be %g and not %g after %d wb_robot_step(s)",
                                i == 0 ? 'X' : i == 1 ? 'Y' : 'Z', e[i], r[i], j);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
