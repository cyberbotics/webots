#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);  // give the controller args

  WbNodeRef old_proto_node = wb_supervisor_node_get_from_def("OLD_PROTO");
  WbNodeRef camera_node = wb_supervisor_node_get_from_proto_def(old_proto_node, "CAMERA_DEVICE");
  WbFieldRef camera_rotation_field = wb_supervisor_node_get_field(camera_node, "rotation");

  wb_robot_step(TIME_STEP);

  const double *rotation = wb_supervisor_field_get_sf_rotation(camera_rotation_field);
  double rotation_expected[] = {0.0, 0.0, 0.0, 0.0};
  ts_assert_doubles_in_delta(4, rotation, rotation_expected, 0.1, "The Camera node should be rotated.");

  ts_send_success();
  return EXIT_SUCCESS;
}
