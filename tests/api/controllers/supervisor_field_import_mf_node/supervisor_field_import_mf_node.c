#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef root = wb_supervisor_node_get_root();
  ts_assert_pointer_not_null(root, "wb_supervisor_node_get_root() failed to return the root node.");

  WbFieldRef field = wb_supervisor_node_get_field(root, "children");
  ts_assert_pointer_not_null(field, "wb_supervisor_node_get_field() failed to find the 'children' field.");

  wb_supervisor_field_import_mf_node(field, 0, "Solid.wbo");

  WbNodeRef floor = wb_supervisor_node_get_from_def("FLOOR");
  ts_assert_pointer_not_null(floor, "wb_supervisor_node_get_from_def() failed to find the 'DEF FLOOR' node.");

  wb_supervisor_field_import_mf_node(field, 0, "Chair.wbo");
  WbNodeRef chair = wb_supervisor_node_get_from_def("CHAIR");
  ts_assert_pointer_not_null(chair, "wb_supervisor_node_get_from_def() failed to find the 'DEF CHAIR' node.");

  WbFieldRef color = wb_supervisor_node_get_field(chair, "color");
  ts_assert_pointer_not_null(color, "wb_supervisor_node_get_field() failed to find the 'color' proto parameter.");

  ts_send_success();
  return EXIT_SUCCESS;
}
