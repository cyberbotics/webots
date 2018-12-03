#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

static const char *temp_world_file_path = "../../worlds/save_joint_proto_parameter_temp.wbt";

static void check_solid_position() {
  const double expected_position[3] = {0, 0.270151, 0.420735};
  const double expected_orientation[9] = {1, 0, 0, 0, 0.540302, -0.841471, 0, 0.841471, 0.540302};
  WbNodeRef solid = wb_supervisor_node_get_from_def("TARGET");
  const double *position = wb_supervisor_node_get_position(solid);
  const double *orientation = wb_supervisor_node_get_orientation(solid);
  ts_assert_doubles_in_delta(3, position, expected_position, 1e-6, "After saving the PROTO parameter Solid position changed.");
  ts_assert_doubles_in_delta(9, orientation, expected_orientation, 1e-6,
                             "After saving the PROTO parameter Solid orientation changed.");
}

int main(int argc, char **argv) {
  wb_robot_init();
  sprintf(ts_test_name, "save_joint_proto_parameter");

  const char *custom_data = wb_robot_get_custom_data();

  if (strcmp(custom_data, "init") == 0) {
    wb_robot_set_custom_data("save");
    wb_supervisor_world_save(temp_world_file_path);
    wb_supervisor_world_load(temp_world_file_path);

  } else {
    ts_setup_done = true;
    ts_notify_controller_status(true);

    check_solid_position();

    ts_send_success();
    return EXIT_SUCCESS;
  }

  wb_robot_cleanup();

  return 0;
}
