#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  const int time_step = wb_robot_get_basic_time_step();

  WbNodeRef cone_node = wb_supervisor_node_get_from_def("CONE");
  WbFieldRef translation_field = wb_supervisor_node_get_field(cone_node, "translation");
  WbFieldRef rotation_field = wb_supervisor_node_get_field(cone_node, "rotation");

  wb_robot_step(46 * time_step);

  /* Ensure the base of the cone has made contact with the ramp. */
  const double *translation_after = wb_supervisor_field_get_sf_vec3f(translation_field);
  const double *rotation_after = wb_supervisor_field_get_sf_rotation(rotation_field);

  ts_assert_double_is_bigger(-rotation_after[3], 2.44, "Cone is not rotated as much as expected.");
  ts_assert_double_is_bigger(translation_after[2], 1.018, "Cone is not as high as expected.");

  ts_send_success();
  return EXIT_SUCCESS;
}
