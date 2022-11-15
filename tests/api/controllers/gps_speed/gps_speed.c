#include <webots/gps.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

void check_speed(WbDeviceTag gps, double v, double vx, double vy, double vz, char *end_msg) {
  double speed = wb_gps_get_speed(gps);
  ts_assert_double_in_delta(speed, v, 0.0001, "The speed value measured by the GPS should be %f and not %lf %s.", v, speed,
                            end_msg);

  const double *r = wb_gps_get_speed_vector(gps);
  ts_assert_vec3_in_delta(r[0], r[1], r[2], vx, vy, vz, 0.0001,
                          "The speed vector value measured by the GPS should be [%f, %f, %f] not [%f, %f, %f] %s.", vx, vy, vz,
                          r[0], r[1], r[2], end_msg);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag gps = wb_robot_get_device("gps");

  check_speed(gps, NAN, NAN, NAN, NAN, "before the device is enabled");
  wb_gps_enable(gps, TIME_STEP);

  check_speed(gps, NAN, NAN, NAN, NAN, "before a wb_robot_step is performed");
  wb_robot_step(TIME_STEP);

  check_speed(gps, NAN, NAN, NAN, NAN, "after one step (because no Physics node)");
  wb_robot_step(TIME_STEP);

  check_speed(gps, 5.0, 0.0, -5.0, 0.0, "after two steps");

  ts_send_success();
  return EXIT_SUCCESS;
}
