#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/file_utils.h"
#include "../../../lib/string_utils.h"
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef kinematic_box = wb_supervisor_node_get_from_def("BOX_GEOMETRY");
  WbFieldRef kinematic_box_size_field = wb_supervisor_node_get_field(kinematic_box, "size");
  WbNodeRef solid = wb_supervisor_node_get_from_def("SOLID");

  // let the 'SOLID' Solid go in sleep mode
  wb_robot_step(320);

  double position[3];
  memcpy(position, wb_supervisor_node_get_position(solid), 3 * sizeof(double));
  const double size[3] = {0.9, 0.4, 0.9};
  wb_supervisor_field_set_sf_vec3f(kinematic_box_size_field, size);

  // let the 'SOLID' Solid move
  wb_robot_step(320);

  const double *new_postion = wb_supervisor_node_get_position(solid);
  ts_assert_vec3_not_in_delta(new_postion[0], new_postion[1], new_postion[2], position[0], position[1], position[2], 0.01,
                              "Solid 'SOLID' didn't moved when changing the size of the box.");

  ts_send_success();
  return EXIT_SUCCESS;
}
