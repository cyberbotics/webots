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
  WbFieldRef coordinate_system_field = wb_supervisor_node_get_field(worldinfo_node, "coordinateSystem");

  wb_robot_step(TIME_STEP);

  const double expected_position[] = {0.0451744, 0.0179479, 0};
  const double expected_enu_position[] = {0, 0.0451744, 0.0179479};
  const double *position = wb_gps_get_values(gps);
  for (int i = 0; i < 3; i++)
    ts_assert_double_in_delta(position[i], expected_position[i], 0.0000001,
                              "The %c value measured by the GPS (NUE) should be %g and not %g",
                              i == 0 ? 'X' : i == 1 ? 'Y' : 'Z', expected_position[i], position[i]);
  wb_supervisor_field_set_sf_string(coordinate_system_field, "ENU");
  wb_robot_step(TIME_STEP);
  position = wb_gps_get_values(gps);
  for (int i = 0; i < 3; i++)
    ts_assert_double_in_delta(position[i], expected_enu_position[i], 0.0000001,
                              "The %c value measured by the GPS (ENU) should be %g and not %g",
                              i == 0 ? 'X' : i == 1 ? 'Y' : 'Z', expected_enu_position[i], position[i]);

  ts_send_success();
  return EXIT_SUCCESS;
}
