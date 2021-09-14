#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);  // give the controller args

  WbNodeRef old_proto_node = wb_supervisor_node_get_from_def("OLD_PROTO");
  WbFieldRef old_proto_rotation_field = wb_supervisor_node_get_field(old_proto_node, "rotation");

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
