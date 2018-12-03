#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef node = wb_supervisor_node_get_from_def("NODE");
  ts_assert_pointer_not_null(node, "wb_supervisor_node_get_from_def(\"NODE\") failed");

  const double *position, *orientation;

  const double NEW_POSITION[3] = {0.2, -0.15, 0.4};
  const double EXPECTED_POSITION[3] = {-0.1, -0.05, 0.4};
  WbFieldRef translation = wb_supervisor_node_get_field(node, "translation");
  wb_supervisor_field_set_sf_vec3f(translation, NEW_POSITION);
  wb_robot_step(TIME_STEP);  // this is necessary to get the absolute position of the node updated by Webots
  position = wb_supervisor_node_get_position(node);

  ts_assert_doubles_in_delta(
    3, position, EXPECTED_POSITION, 0.0001,
    "wb_supervisor_node_get_position() did not return the expected set value ({%f, %f, %f} instead of {%f, %f, %f})",
    position[0], position[1], position[2], EXPECTED_POSITION[0], EXPECTED_POSITION[1], EXPECTED_POSITION[2]);

  const double EXPECTED_ROTATION_MATRIX3[9] = {-0.5, 0.866, 0.0, -0.866, -0.5, 0.0, 0.0, 0.0, 1.0};
  const double NEW_ROTATION_FIELD_VALUE[4] = {0.0, 0.0, 1.0, M_PI / 3.0};

  WbFieldRef rotation = wb_supervisor_node_get_field(node, "rotation");
  wb_supervisor_field_set_sf_rotation(rotation, NEW_ROTATION_FIELD_VALUE);
  wb_robot_step(TIME_STEP);  // this is necessary to get the absolute rotation of the node updated by Webots
  orientation = wb_supervisor_node_get_orientation(node);

  ts_assert_doubles_in_delta(9, orientation, EXPECTED_ROTATION_MATRIX3, 0.001,
                             "wb_supervisor_node_get_orientation() did not return the expected changed values.");

  ts_send_success();
  return EXIT_SUCCESS;
}
