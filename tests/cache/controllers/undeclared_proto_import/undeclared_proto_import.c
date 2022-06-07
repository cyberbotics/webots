#include <stdio.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, const char *argv[]) {
  ts_setup(argv[0]);

  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
  wb_supervisor_field_import_mf_node_from_string(root_children_field, 4, "DEF PANDA Panda { }");

  wb_robot_step(TIME_STEP);

  WbNodeRef panda_node = wb_supervisor_node_get_from_def("PANDA");


  ts_send_success();
  return EXIT_SUCCESS;
}
