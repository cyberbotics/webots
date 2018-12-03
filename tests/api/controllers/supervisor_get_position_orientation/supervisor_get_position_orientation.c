#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef node = wb_supervisor_node_get_from_def("NODE");
  ts_assert_pointer_not_null(node, "wb_supervisor_node_get_from_def(\"NODE\") failed");

  WbNodeRef node2 = wb_supervisor_node_get_from_def("ROBOT.JOINT.SOLID.NODE");
  ts_assert_pointer_not_null(node2, "wb_supervisor_node_get_from_def(\"ROBOT.JOINT.SOLID.NODE\") failed");

  if (node != node2)
    ts_assert_boolean_equal(
      false,
      "wb_supervisor_node_get_from_def(\"ROBOT.JOINT.SOLID.NODE\") failed, reference to node and node2 should be equal.");

  const double *position = wb_supervisor_node_get_position(node);
  const double POSITION[] = {0.1, -0.2, 0.3};

  ts_assert_doubles_in_delta(3, position, POSITION, 0.000001,
                             "wb_supervisor_node_get_position() did not return the expected values.");

  const double *orientation = wb_supervisor_node_get_orientation(node);
  const double ROTATION[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  ts_assert_doubles_in_delta(9, orientation, ROTATION, 0.0001,
                             "wb_supervisor_node_get_orientation() did not return the expected values.");

  ts_send_success();
  return EXIT_SUCCESS;
}
