#include <webots/gps.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  const double *r;
  const double e[3] = {0.0, -5.0, 0.0};
  WbDeviceTag gps = wb_robot_get_device("gps");

  double speed = wb_gps_get_speed(gps);
  ts_assert_double_equal(speed, NAN,
                         "The speed value measured by the GPS should be NaN and not %lf before the device is enabled", speed);

  r = wb_gps_get_speed_vector(gps);
  ts_assert_vec3_equal(
    r[0], r[1], r[2], NAN, NAN, NAN,
    "The speed vector value measured by the GPS should be [NaN, NaN, NaN] not [%f, %f, %f] before the device is enabled", r[0],
    r[1], r[2]);

  wb_gps_enable(gps, TIME_STEP);

  speed = wb_gps_get_speed(gps);
  ts_assert_double_equal(
    speed, NAN, "The speed value measured by the GPS should be NaN and not %lf before a wb_robot_step is performed", speed);

  r = wb_gps_get_speed_vector(gps);
  ts_assert_vec3_equal(
    r[0], r[1], r[2], NAN, NAN, NAN,
    "The speed vector value measured by the GPS should be [NaN, NaN, NaN] not [%f, %f, %f] before a wb_robot_step is performed",
    r[0], r[1], r[2]);

  wb_robot_step(2 * TIME_STEP);

  speed = wb_gps_get_speed(gps);
  ts_assert_double_in_delta(speed, 5.0, 0.0001, "The speed value measured by the GPS should be 5.0 and not %lf after two steps",
                            speed);

  r = wb_gps_get_speed_vector(gps);
  ts_assert_vec3_in_delta(
    r[0], r[1], r[2], e[0], e[1], e[2], 0.0001,
    "The speed vector value measured by the GPS should be [%f, %f, %f] not [%f, %f, %f] after %d wb_robot_step(s)", e[0], e[1],
    e[2], r[0], r[1], r[2], 2);

  ts_send_success();
  return EXIT_SUCCESS;
}
