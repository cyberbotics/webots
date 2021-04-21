/*
 * Description:
 */

#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // WbNodeRef root = wb_supervisor_node_get_root();
  // WbFieldRef rootChildren = wb_supervisor_node_get_field(root, "children");
  WbNodeRef robot_visible_node = wb_supervisor_node_get_from_def("ROBOT_VISIBLE_PROTO");
  WbFieldRef rotation_field = wb_supervisor_node_get_field(robot_visible_node, "rotation");
  WbFieldRef translation_field = wb_supervisor_node_get_field(robot_visible_node, "translation");
  WbNodeRef joint_parameters_node = wb_supervisor_node_get_from_def("JOINT_PARAMETERS");
  WbFieldRef position_field = wb_supervisor_node_get_field(joint_parameters_node, "position");

  WbNodeRef visible_solid_node = wb_supervisor_node_get_from_def("VISIBLE_SOLID_BODY");
  WbNodeRef nested_proto = wb_supervisor_node_get_from_def("PROTO_HINGE_JOINT");
  WbNodeRef invisible_solid_node = wb_supervisor_node_get_from_proto_def(nested_proto, "INVISIBLE_SOLID_BODY");

  // also ensure that the visible one is being refreshed in the interface
  // const double *previous_rotation = wb_supervisor_field_get_sf_rotation(rotation_field);
  // const double *previous_translation = wb_supervisor_field_get_sf_vec3f(translation_field);
  // const double previous_position = wb_supervisor_field_get_sf_float(position_field);

  for (int i = 0; i < 10; ++i) {
    wb_robot_step(TIME_STEP);
    const double *visible_orientation = wb_supervisor_node_get_orientation(visible_solid_node);
    const double *invisible_orientation = wb_supervisor_node_get_orientation(invisible_solid_node);
    for (int j = 0; j < 9; ++j) {
      ts_assert_double_equal(invisible_orientation[j], visible_orientation[j],
                             "The orientation of the robots with visible and invisible nodes should be the same but isn't");
    }

    /*
    if (i > 0) {
      const double *rotation = wb_supervisor_field_get_sf_rotation(rotation_field);
      const double *translation = wb_supervisor_field_get_sf_vec3f(translation_field);
      const double position = wb_supervisor_field_get_sf_float(position_field);

      ts_assert_double_not_equal(position, previous_position, "Visible position field isn't refreshing");

      previous_rotation = rotation;
      previous_translation = translation;
      previous_position = position;
    }
    */
  }

  // ensure nested proto with visible field doesn't get deleted (i.e check if exposed parameter refreshes)
  // const double *visible_hinge_rotation = wb_supervisor_field_get_sf_rotation(hinge_rotation_field);
  // ts_assert_vec3_not_in_delta(hinge_rotation[0], hinge_rotation[1], hinge_rotation[2], 1, 0, 0, 1e-10,
  //                            "Rotation field should've refreshed but didn't.");

  // both the invisible and visible one should've rotate by the same amount
  // const double *invisible_hinge_rotation = wb_supervisor_field_get_sf_rotation(hinge_rotation_field);

  ts_send_success();
  return EXIT_SUCCESS;
}
