#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

const double expectedPos1[] = {1.353466, -1.088782, -0.527743};
const double expectedPos2[] = {-0.798742, -0.927224, 0.699450};

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef node1 = wb_supervisor_node_get_from_def("BOX1");
  WbNodeRef node2 = wb_supervisor_node_get_from_def("BOX2");
  WbFieldRef translation1 = wb_supervisor_node_get_field(node1, "translation");
  WbFieldRef translation2 = wb_supervisor_node_get_field(node2, "translation");

  int c;
  for (c = 0; c < 40; ++c)
    wb_robot_step(TIME_STEP);

  const double *pos1 = wb_supervisor_field_get_sf_vec3f(translation1);
  const double *pos2 = wb_supervisor_field_get_sf_vec3f(translation2);

  for (c = 0; c < 3; c++) {
    ts_assert_double_in_delta(pos1[c], expectedPos1[c], 0.001, "The box1 is not at the position expected.");
    ts_assert_double_in_delta(pos2[c], expectedPos2[c], 0.001, "The box2 is not at the position expected.");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
