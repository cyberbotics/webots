#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, const char *argv[]) {
  ts_setup(argv[0]);

  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");

  // import case: declared as EXTERNPROTO and is an official PROTO
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, "DEF NAO Nao { }");
  WbNodeRef nao_node = wb_supervisor_node_get_from_def("NAO");

  // import case: declared as EXTERNPROTO but is not an official PROTO
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, "DEF DECLARED_PROTO DeclaredProto { }");
  WbNodeRef custom_proto_node = wb_supervisor_node_get_from_def("DECLARED_PROTO");

  // import case: not declared as EXTERNPROTO but is an official PROTO
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, "DEF DARWIN_OP Darwin-op { }");
  WbNodeRef darwin_node = wb_supervisor_node_get_from_def("DARWIN_OP");

  // import case: not declared as EXTERNPROTO and is not an official PROTO
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, "DEF NONEXISTANT_PROTO UndeclaredProto { }");
  WbNodeRef non_existant_node = wb_supervisor_node_get_from_def("UNDECLARED_PROTO");

  wb_robot_step(TIME_STEP);

  ts_assert_pointer_not_null(nao_node, "Importing of 'Nao' robot should work but does not.");
  ts_assert_pointer_not_null(custom_proto_node, "Importing of 'CustomProto' robot should work but does not.");
  ts_assert_pointer_null(darwin_node, "Importing of 'Darwin-Op' robot should not be possible.");
  ts_assert_pointer_null(non_existant_node, "Importing of 'NonExistantProto' robot should not be possible.");

  ts_send_success();
  return EXIT_SUCCESS;
}
