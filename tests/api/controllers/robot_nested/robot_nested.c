#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  const double targetVelocity[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  WbNodeRef childRobot = wb_supervisor_node_get_from_def("CHILD_ROBOT");
  WbNodeRef supervisor = wb_supervisor_node_get_self();
  const double *childPosition;

  ts_assert_int_equal(wb_robot_get_device("gps"), 0, "The child robot's gps device should not be found by the parent robot");

  childPosition = wb_supervisor_node_get_position(supervisor);
  wb_supervisor_node_set_velocity(supervisor, targetVelocity);

  wb_robot_step(TIME_STEP);

  childPosition = wb_supervisor_node_get_position(childRobot);
  ts_assert_double_equal(childPosition[0], 0.0, "The child's tranlation in x should be 0");
  ts_assert_double_equal(childPosition[1], 0.1, "The child's tranlation in x should be 0.1");
  ts_assert_double_equal(childPosition[2], 0.016, "The child's tranlation in x should be 0.016");

  ts_send_success();
  return EXIT_SUCCESS;
}
