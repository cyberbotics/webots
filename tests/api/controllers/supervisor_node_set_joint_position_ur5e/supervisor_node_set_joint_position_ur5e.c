/*
 * Description:  Test Supervisor API: wb_supervisor_node_set_joint_position for internal PROTO joints
 *
 */

#include <webots/gps.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  wb_robot_step(TIME_STEP);

  const double *initial_gps_value = wb_gps_get_values(gps);
  printf("initial_gps_value %f %f %f\n", initial_gps_value[0], initial_gps_value[1], initial_gps_value[2]);
  ts_assert_vec3_in_delta(initial_gps_value[0], initial_gps_value[1], initial_gps_value[2], 0.8170, 0.0628, -0.2340, 0.0001,
                          "Wrong initial position of UR5e tool.");

  WbDeviceTag shoulder_lift_motor = wb_robot_get_device("shoulder_lift_joint");
  WbNodeRef shoulder_lift_motor_node = wb_supervisor_node_get_from_device(shoulder_lift_motor);
  WbNodeRef shoulder_lift_joint_node = wb_supervisor_node_get_parent_node(shoulder_lift_motor_node);
  wb_supervisor_node_set_joint_position(shoulder_lift_joint_node, -1.55, 1);

  wb_robot_step(TIME_STEP);

  const double *new_gps_value = wb_gps_get_values(gps);
  printf("new_gps_value %f %f %f\n", new_gps_value[0], new_gps_value[1], new_gps_value[2]);
  ts_assert_vec3_in_delta(new_gps_value[0], new_gps_value[1], new_gps_value[2], 0.1170, 0.9776, -0.2340, 0.0001,
                          "Wrong initial position of UR5e tool.");

  ts_send_success();
  return EXIT_SUCCESS;
}
