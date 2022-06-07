#include <stdio.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, const char *argv[]) {
  ts_setup(argv[0]);

  const bool is_declared = strcmp(argv[1], "declared") == 0 ? true : false;
  const bool is_official = strcmp(argv[2], "official") == 0 ? true : false;

  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");

  wb_robot_step(TIME_STEP);

  WbNodeRef node;
  if (is_declared && is_official) {
    wb_supervisor_field_import_mf_node_from_string(root_children_field, 4, "DEF NAO Nao { }");
    wb_robot_step(TIME_STEP);
    node = wb_supervisor_node_get_from_def("NAO");
  } else if (is_declared && !is_official) {
    wb_supervisor_field_import_mf_node_from_string(root_children_field, 4, "DEF CUSTOM_PROTO CustomProto { }");
    wb_robot_step(TIME_STEP);
    node = wb_supervisor_node_get_from_def("CUSTOM_PROTO");
  } else if (!is_declared && is_official) {
    wb_supervisor_field_import_mf_node_from_string(root_children_field, 4, "DEF NED Ned { }");
    wb_robot_step(TIME_STEP);
    node = wb_supervisor_node_get_from_def("NED");
  } else if (!is_declared && !is_official) {
    wb_supervisor_field_import_mf_node_from_string(root_children_field, 4, "DEF NONEXISTANT_PROTO NonExistantProto { }");
    wb_robot_step(TIME_STEP);
    node = wb_supervisor_node_get_from_def("NONEXISTANT_PROTO");
  } else {
    node = NULL;
    ts_assert_boolean_equal(0, "Unknown configuration.");
  }

  wb_robot_step(TIME_STEP);

  if (is_declared && is_official)
    ts_assert_pointer_not_null(node, "Importing of 'declared & official' PROTO should work but does not.");
  else if (!is_declared && is_official)
    ts_assert_pointer_not_null(node, "Importing of 'undeclared & official' PROTO should work but does not.");
  else if (is_declared && !is_official)
    ts_assert_pointer_not_null(node, "Importing of 'declared & unofficial' PROTO should work but does not.");
  else
    ts_assert_pointer_null(node, "It should not be possible to import a 'undeclared & unofficial' PROTO.");

  ts_send_success();
  return EXIT_SUCCESS;
}
