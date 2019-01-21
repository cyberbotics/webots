#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int time_step = wb_robot_get_basic_time_step();

  WbNodeRef ball_joint_robot = wb_supervisor_node_get_from_def("BALL_JOINT_ROBOT");
  WbNodeRef ball_joint_robot_kinematic = wb_supervisor_node_get_from_def("BALL_JOINT_ROBOT_KINEMATIC");
  WbNodeRef hinge_joint_robot = wb_supervisor_node_get_from_def("HINGE_JOINT_ROBOT");
  WbNodeRef hinge_joint_robot_kinematic = wb_supervisor_node_get_from_def("HINGE_JOINT_ROBOT_KINEMATIC");
  WbNodeRef ball_joint_marker = wb_supervisor_node_get_from_def("BALL_JOINT_MARKER");
  WbNodeRef ball_joint_marker_kinematic = wb_supervisor_node_get_from_def("BALL_JOINT_MARKER_KINEMATIC");
  WbNodeRef hinge_joint_marker = wb_supervisor_node_get_from_def("HINGE_JOINT_MARKER");
  WbNodeRef hinge_joint_marker_kinematic = wb_supervisor_node_get_from_def("HINGE_JOINT_MARKER_KINEMATIC");

  while (wb_robot_step(time_step) != -1.0 && wb_robot_get_time() < 6.0) {
    const double *ball_joint_robot_pos = wb_supervisor_node_get_position(ball_joint_robot);
    const double *ball_joint_robot_kinematic_pos = wb_supervisor_node_get_position(ball_joint_robot_kinematic);
    const double *hinge_joint_robot_pos = wb_supervisor_node_get_position(hinge_joint_robot);
    const double *hinge_joint_robot_kinematic_pos = wb_supervisor_node_get_position(hinge_joint_robot_kinematic);
    const double *ball_joint_marker_pos = wb_supervisor_node_get_position(ball_joint_marker);
    const double *ball_joint_marker_kinematic_pos = wb_supervisor_node_get_position(ball_joint_marker_kinematic);
    const double *hinge_joint_marker_pos = wb_supervisor_node_get_position(hinge_joint_marker);
    const double *hinge_joint_marker_kinematic_pos = wb_supervisor_node_get_position(hinge_joint_marker_kinematic);

    ts_assert_vec3_in_delta(
      hinge_joint_marker_pos[0] - hinge_joint_robot_pos[0], hinge_joint_marker_pos[1] - hinge_joint_robot_pos[1],
      hinge_joint_marker_pos[2] - hinge_joint_robot_pos[2], ball_joint_marker_pos[0] - ball_joint_robot_pos[0],
      ball_joint_marker_pos[1] - ball_joint_robot_pos[1], ball_joint_marker_pos[2] - ball_joint_robot_pos[2],
      0.02,  // small difference due to the dummy intermediate solids in the
             // hinges case
      "Positions of the hinges and ball joints targets differ.");

    ts_assert_vec3_in_delta(hinge_joint_marker_pos[0] - hinge_joint_robot_pos[0],
                            hinge_joint_marker_pos[1] - hinge_joint_robot_pos[1],
                            hinge_joint_marker_pos[2] - hinge_joint_robot_pos[2],
                            hinge_joint_marker_kinematic_pos[0] - hinge_joint_robot_kinematic_pos[0],
                            hinge_joint_marker_kinematic_pos[1] - hinge_joint_robot_kinematic_pos[1],
                            hinge_joint_marker_kinematic_pos[2] - hinge_joint_robot_kinematic_pos[2],
                            0.02,  // small difference due to the dummy intermediate solids in the
                                   // hinges case
                            "Positions of the kinematic and dynamic hinge joints targets differ.");

    ts_assert_vec3_in_delta(ball_joint_marker_pos[0] - ball_joint_robot_pos[0],
                            ball_joint_marker_pos[1] - ball_joint_robot_pos[1],
                            ball_joint_marker_pos[2] - ball_joint_robot_pos[2],
                            ball_joint_marker_kinematic_pos[0] - ball_joint_robot_kinematic_pos[0],
                            ball_joint_marker_kinematic_pos[1] - ball_joint_robot_kinematic_pos[1],
                            ball_joint_marker_kinematic_pos[2] - ball_joint_robot_kinematic_pos[2],
                            0.02,  // small difference due to the dummy intermediate solids in the
                                   // hinges case
                            "Positions of the kinematic and dynamic ball joints targets differ.");
  }

  const double *ball_joint_marker_pos = wb_supervisor_node_get_position(ball_joint_marker);
  ts_assert_vec3_in_delta(ball_joint_marker_pos[0], ball_joint_marker_pos[1], ball_joint_marker_pos[2], 0.129531, 0.155783,
                          0.223086, 0.001, "Final position of the targets not at the expected position.");

  ts_send_success();
  return EXIT_SUCCESS;
}
