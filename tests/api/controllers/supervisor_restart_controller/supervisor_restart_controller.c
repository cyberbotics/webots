/*
 * Description:  Test supervisor function that resets the robot controller field
 *               to the same value.
 *               The controller has to be restarted automatically and the robot
 *               should start moving forward.
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, char **argv) {
  const double *normalRobotPosition, *protoRobotPosition;
  double normalRobotPrevZPosition, protoRobotPrevZPosition;

  ts_setup(argv[0]);

  WbNodeRef normalRobot = wb_supervisor_node_get_from_def("MY_BOT");
  WbNodeRef protoRobot = wb_supervisor_node_get_from_def("RESETTER");

  wb_robot_step(TIME_STEP);

  normalRobotPosition = wb_supervisor_node_get_position(normalRobot);
  protoRobotPosition = wb_supervisor_node_get_position(protoRobot);
  ts_assert_boolean_equal(normalRobotPosition[2] < 0.0,
                          "The base node controller is not started correctly and the base node robot did not move forwards.");
  ts_assert_boolean_equal(protoRobotPosition[2] < 0.0,
                          "The PROTO controller is not started correctly and the base node robot did not move forwards.");

  wb_robot_step(10 * TIME_STEP);

  normalRobotPosition = wb_supervisor_node_get_position(normalRobot);
  normalRobotPrevZPosition = normalRobotPosition[2];

  protoRobotPosition = wb_supervisor_node_get_position(protoRobot);
  protoRobotPrevZPosition = protoRobotPosition[2];

  // reset controller
  wb_supervisor_node_restart_controller(normalRobot);
  wb_supervisor_node_restart_controller(protoRobot);

  wb_robot_step(2 * TIME_STEP);

  normalRobotPosition = wb_supervisor_node_get_position(normalRobot);
  printf("Base Node Robot Position %f %f\n", normalRobotPrevZPosition, normalRobotPosition[2]);
  ts_assert_boolean_equal(
    normalRobotPrevZPosition > normalRobotPosition[2],
    "The base node robot's controller is not restarted correctly and the base node robot did not move forwards.");

  protoRobotPosition = wb_supervisor_node_get_position(protoRobot);
  printf("PROTO Robot Position %f %f\n", protoRobotPrevZPosition, protoRobotPosition[2]);
  ts_assert_boolean_equal(
    protoRobotPrevZPosition > protoRobotPosition[2],
    "The PROTO robot's controller is not restarted correctly and the base node robot did not move forwards.");

  ts_send_success();
  return EXIT_SUCCESS;
}
