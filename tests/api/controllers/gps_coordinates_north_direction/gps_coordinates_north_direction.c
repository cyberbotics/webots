#include <webots/gps.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  WbNodeRef worldinfo_node = wb_supervisor_node_get_from_def("WORLDINFO");
  WbFieldRef north_direction_field = wb_supervisor_node_get_field(worldinfo_node, "northDirection");

  wb_robot_step(TIME_STEP);

  const double *position = wb_gps_get_values(gps);
  ts_assert_double_in_delta(position[2], 0.0, 0.0001, "The altitude measured by the GPS should be 0 and not %f", position[2]);

  double old_latitude = position[0];
  double old_longitude = position[1];

  const double new_direction[3] = {0, 0, 1};
  wb_supervisor_field_set_sf_vec3f(north_direction_field, new_direction);
  wb_robot_step(TIME_STEP);

  position = wb_gps_get_values(gps);
  ts_assert_double_in_delta(position[2], 0.0, 0.0001, "The altitude measured by the GPS should be 0 and not %f", position[2]);

  // after changing the northDirection, the latitude and longitude should be inversed
  ts_assert_double_in_delta(position[0], old_longitude, 0.001,
                            "The latitude and longitude have not been correctly inversed after changing the northDirection.");
  ts_assert_double_in_delta(position[1], -old_latitude, 0.001,
                            "The latitude and longitude have not been correctly inversed after changing the northDirection.");

  ts_send_success();
  return EXIT_SUCCESS;
}
