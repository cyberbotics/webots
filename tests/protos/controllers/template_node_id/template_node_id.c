#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[1]);  // give the controller args

  // let the time for the robot controller to copy the args in customData field
  wb_robot_step(5 * TIME_STEP);

  WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");
  WbFieldRef custom_data_field = wb_supervisor_node_get_field(robot, "customData");
  WbFieldRef procedural_field = wb_supervisor_node_get_field(robot, "proceduralField");

  const char *field_content = wb_supervisor_field_get_sf_string(custom_data_field);
  int id;
  sscanf(field_content, "%d", &id);

  ts_assert_int_equal(
    id, wb_supervisor_node_get_id(robot),
    "The 'id' returned by 'wb_supervisor_node_get_id' does not match the one received by the PROTO as 'context.id'.");

  wb_supervisor_field_set_sf_string(custom_data_field, "");
  wb_supervisor_field_set_sf_string(procedural_field, "");
  wb_robot_step(5 * TIME_STEP);

  const char *new_field_content = wb_supervisor_field_get_sf_string(custom_data_field);
  int new_id;
  sscanf(new_field_content, "%d", &new_id);

  ts_assert_int_equal(id, new_id, "The 'id' received by the PROTO as 'context.id' has changed when the PROTO was regenerated.");

  ts_assert_int_equal(
    new_id, wb_supervisor_node_get_id(robot),
    "The 'id' returned by 'wb_supervisor_node_get_id' does not match the one received by the PROTO as 'context.id'.");

  WbNodeRef root_node = wb_supervisor_node_get_root();
  ts_assert_int_equal(0, wb_supervisor_node_get_id(root_node), "The 'id' of the root node should be '0'.");

  WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
  ts_assert_pointer_not_null(root_children_field, "Can't find 'children' field of root node.");

  ts_send_success();
  return EXIT_SUCCESS;
}
