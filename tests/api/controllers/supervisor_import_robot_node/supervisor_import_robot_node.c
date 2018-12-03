/*
 * Description:  Test supervisor import function in case of a robot object.
 *               The controller of the imported robot has to be started
 *               automatically and after some step the position of the robot
 *               has to be different from the inial one.
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  const double initialPosition[3] = {0.0, 0.0, -0.104};

  WbNodeRef root = wb_supervisor_node_get_root();
  WbFieldRef rootChildren = wb_supervisor_node_get_field(root, "children");

  wb_robot_step(2 * TIME_STEP);

  // import robot object
  wb_supervisor_field_import_mf_node(rootChildren, -1, "MyBot.wbo");

  // check imported robot position
  WbNodeRef robot = wb_supervisor_node_get_from_def("MY_BOT");
  ts_assert_boolean_equal(robot != NULL, "The robot node has not been imported correctly");

  const double *position = wb_supervisor_node_get_position(robot);
  ts_assert_doubles_in_delta(3, position, initialPosition, 0.0001, "The robot has not been imported at the correct position");

  wb_robot_step(4 * TIME_STEP);

  // check that robot moves forwards
  double previousZ = position[2];
  position = wb_supervisor_node_get_position(robot);
  ts_assert_boolean_equal(position[2] < (previousZ - 0.02), "The controller of the imported robot is not started correctly");

  ts_send_success();
  return EXIT_SUCCESS;
}
