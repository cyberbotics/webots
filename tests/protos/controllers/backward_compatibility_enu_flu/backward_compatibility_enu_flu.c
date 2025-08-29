#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);  // give the controller args

  WbNodeRef old_proto_node = wb_supervisor_node_get_from_def("OLD_PROTO");
  WbNodeRef cylinder_node = wb_supervisor_node_get_from_proto_def(old_proto_node, "CYLINDER_POSE");

  wb_robot_step(TIME_STEP);
  const double *camera_pose = wb_supervisor_node_get_pose(old_proto_node, NULL);
  const double camera_pose_expected[] = {1, 0, 0};
  const double camera_pose_x_theta[] = {camera_pose[0], camera_pose[4], camera_pose[8]};
  ts_assert_doubles_in_delta(3, camera_pose_x_theta, camera_pose_expected, 0.1,
                             "The Camera node should be looking towards the box (got %lf %lf %lf).", camera_pose_x_theta[0],
                             camera_pose_x_theta[1], camera_pose_x_theta[2]);

  const double *cylinder_pose = wb_supervisor_node_get_pose(cylinder_node, NULL);
  const double cylinder_pose_expected[] = {-1, 0, 0};
  const double cylinder_pose_z_theta[] = {cylinder_pose[2], cylinder_pose[6], cylinder_pose[10]};
  ts_assert_doubles_in_delta(3, cylinder_pose_z_theta, cylinder_pose_expected, 0.1,
                             "The Cylinder node should be looking towards the box (got %lf %lf %lf).", cylinder_pose_z_theta[0],
                             cylinder_pose_z_theta[1], cylinder_pose_z_theta[2]);

  wb_robot_step(TIME_STEP);
  ts_send_success();
  return EXIT_SUCCESS;
}
