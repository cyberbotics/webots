#include <webots/device.h>
#include <webots/gps.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

static void check_speed(WbDeviceTag gps, double v, double vx, double vy, double vz, char *end_msg) {
  const double speed = wb_gps_get_speed(gps);
  ts_assert_double_in_delta(speed, v, 0.0001, "The speed value measured by the GPS \"%s\" should be %f and not %lf %s.",
                            wb_device_get_name(gps), v, speed, end_msg);

  const double *r = wb_gps_get_speed_vector(gps);
  ts_assert_vec3_in_delta(r[0], r[1], r[2], vx, vy, vz, 0.0001,
                          "The speed vector value measured by the GPS \"%s\" should be [%f, %f, %f] not [%f, %f, %f] %s.",
                          wb_device_get_name(gps), vx, vy, vz, r[0], r[1], r[2], end_msg);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  const int time_step = wb_robot_get_basic_time_step();

  WbDeviceTag gps_with_physics = wb_robot_get_device("gps_with_physics");
  WbDeviceTag gps_without_physics = wb_robot_get_device("gps_without_physics");

  check_speed(gps_with_physics, NAN, NAN, NAN, NAN, "before the device is enabled");
  wb_gps_enable(gps_with_physics, time_step);
  check_speed(gps_without_physics, NAN, NAN, NAN, NAN, "before the device is enabled");
  wb_gps_enable(gps_without_physics, time_step);

  check_speed(gps_with_physics, NAN, NAN, NAN, NAN, "before a wb_robot_step is performed");
  check_speed(gps_without_physics, NAN, NAN, NAN, NAN, "before a wb_robot_step is performed");
  wb_robot_step(time_step);

  check_speed(gps_with_physics, 5.0, 0.0, 0.0, -5.0, "after one step (because got a Physics node)");
  check_speed(gps_without_physics, NAN, NAN, NAN, NAN, "after one step (because no Physics node)");
  wb_robot_step(time_step);

  check_speed(gps_with_physics, 5.0, 0.0, 0.0, -5.0, "after two steps");
  check_speed(gps_without_physics, 5.0, -5.0, 0.0, 0.0, "after two steps");

  ts_send_success();
  return EXIT_SUCCESS;
}
