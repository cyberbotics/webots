#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);  // give the controller args

  WbNodeRef node = wb_supervisor_node_get_from_def("VIEWPOINT");
  WbFieldRef position_field = wb_supervisor_node_get_field(node, "position");
  const double *position = wb_supervisor_field_get_sf_vec3f(position_field);
  double initial_position[3];
  memcpy(initial_position, position, 3 * sizeof(double));
  double expected_position[3];
  expected_position[0] = initial_position[0];
  expected_position[1] = initial_position[1] - 3.26275;
  expected_position[2] = initial_position[2];

  wb_robot_step(25 * TIME_STEP);

  position = wb_supervisor_field_get_sf_vec3f(position_field);
  ts_assert_vec3_in_delta(position[0], position[1], position[2], expected_position[0], expected_position[1],
                          expected_position[2], 0.01, "The Viewpoint position is not the expected one");

  ts_send_success();

  return EXIT_SUCCESS;
}
