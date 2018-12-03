/*
 * Description:  Test Supervisor API
 *               This file contains some tests of the methods relative to fields,
 *               i.e. functions starting with wb_supervisor_field_* for:
 *                  SF_FLOAT
 *                  MF_COLOR
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef robot_node, internal_node;
  WbFieldRef normal_field, hidden_field, internal_field;

  wb_robot_step(TIME_STEP);

  // check that root can be found
  robot_node = wb_supervisor_node_get_from_def("ROBOT");
  ts_assert_pointer_not_null(robot_node, "ROBOT node is not found");

  // check that a normal accessible field can be found
  normal_field = wb_supervisor_node_get_field(robot_node, "translation");
  ts_assert_pointer_not_null(normal_field, "'translation' field is not found");

  // check that a hidden accessible field can be found
  hidden_field = wb_supervisor_node_get_field(robot_node, "rotation");
  ts_assert_pointer_not_null(hidden_field, "'rotation' hidden field is not found");

  // check that an internal field can't be found
  internal_field = wb_supervisor_node_get_field(robot_node, "scale");
  ts_assert_pointer_null(internal_field, "'scale' internal field found");

  // check that an internal node can't be found using get_from_def
  internal_node = wb_supervisor_node_get_from_def("INTERNAL_SOLID");
  ts_assert_pointer_null(internal_node, "'INTERNAL_SOLID' internal node found");

  ts_send_success();
  return EXIT_SUCCESS;
}
