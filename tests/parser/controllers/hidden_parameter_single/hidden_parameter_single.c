#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  double expectedInitPos[3] = {0.1, 0.1, 0.23};
  double angle = 0.47;
  double cosAngle = cos(angle);
  double sinAngle = sin(angle);
  double expectedMatrix[9] = {1.0, 0.0, 0.0, 0.0, cosAngle, -sinAngle, 0.0, sinAngle, cosAngle};
  ts_setup(argv[0]);
  ts_disable_output_log();

  WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");

  ts_assert_doubles_equal(3, wb_supervisor_node_get_position(robot), expectedInitPos,
                          "The hidden translation has not been set correctly");

  ts_assert_doubles_equal(9, wb_supervisor_node_get_orientation(robot), expectedMatrix,
                          "The hidden rotation has not been set correctly");

  wb_robot_step(TIME_STEP);

  const double *position = wb_supervisor_node_get_position(robot);
  ts_assert_boolean_equal((expectedInitPos[1] + 0.015) < position[1], "The hidden linear velocity has not been set correctly");

  ts_send_success();
  return EXIT_SUCCESS;
}
