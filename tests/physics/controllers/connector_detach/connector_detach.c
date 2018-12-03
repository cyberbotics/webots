#include <stdio.h>
#include <webots/connector.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  double previous_distance = 0;
  ts_setup(argv[0]);

  WbNodeRef active_robot = wb_supervisor_node_get_self();
  WbNodeRef passive_robot = wb_supervisor_node_get_from_def("PASSIVE_ROBOT");

  WbDeviceTag connector = wb_robot_get_device("connector");
  wb_connector_enable_presence(connector, TIME_STEP);

  wb_robot_step(TIME_STEP);
  {
    const double *p1 = wb_supervisor_node_get_position(active_robot);
    const double *p2 = wb_supervisor_node_get_position(passive_robot);
    previous_distance = p1[0] - p2[0];
  }
  ts_assert_boolean_equal(wb_connector_get_presence(connector) == 1,
                          "Connectors are expected to be in contact at the beginning.");

  wb_connector_lock(connector);

  wb_robot_step(TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    double time = wb_robot_get_time();
    if (time > 0.736) {
      ts_assert_boolean_equal(false, "Connectors should not be in contact after applying a force of 2.1 N.");
      break;
    }
    if (wb_connector_get_presence(connector) == 0) {
      if (time < 0.736)
        ts_assert_boolean_equal(false, "Connectors should not be detached when applying a force < 2.1 N.");
      break;
    }
  }

  {
    const double *p1 = wb_supervisor_node_get_position(active_robot);
    const double *p2 = wb_supervisor_node_get_position(passive_robot);
    ts_assert_double_is_bigger((p1[0] - p2[0]), previous_distance + 0.01,
                               "Robots are expected to be far away after applying enough force.");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
