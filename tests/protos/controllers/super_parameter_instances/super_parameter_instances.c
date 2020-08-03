/*
 * Regression test for issue #6059
 */

#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);  // give the controller args

  WbNodeRef node = wb_supervisor_node_get_from_def("SENSOR_SLOT");
  WbFieldRef children_field = wb_supervisor_node_get_field(node, "children");

  // Webots should not crash when inserting subnodes,
  // this means that the nodes in mSuperParametersAtInitialization list are ignored during finalization
  wb_supervisor_field_import_mf_node_from_string(children_field, 0, "Accelerometer { }");
  wb_robot_step(TIME_STEP);

  wb_supervisor_field_import_mf_node_from_string(children_field, 1, "VelodyneVLP-16 { }");
  wb_robot_step(TIME_STEP);

  wb_supervisor_field_import_mf_node_from_string(
    children_field, 1, "Servo { translation 0 1 0 children [ DerivedColoredBoxShape { boxSize 0.5 0.5 0.5 } ] }");
  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
