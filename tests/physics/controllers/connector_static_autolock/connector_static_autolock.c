#include <stdio.h>

#include <webots/connector.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef solid = wb_supervisor_node_get_from_def("SOLID");

  WbDeviceTag connector = wb_robot_get_device("connector");
  wb_connector_enable_presence(connector, TIME_STEP);

  wb_robot_step(TIME_STEP);

  ts_assert_boolean_equal(wb_connector_get_presence(connector) == 1,
                          "Connectors are expected to be in contact at the beginning.");

  wb_connector_lock(connector);

  wb_robot_step(TIME_STEP);

  for (int i = 0; i < 100; ++i)
    wb_robot_step(TIME_STEP);

  const double *position = wb_supervisor_node_get_position(solid);
  ts_assert_vec3_in_delta(position[0], position[1], position[2], 0.25, 0.2, 0.0, 0.001,
                          "Solid is not at the expected position ([%lf, %lf, %lf] instead of [0.25, 0.2, 0.0]).", position[0],
                          position[1], position[2]);

  ts_send_success();
  return EXIT_SUCCESS;
}
