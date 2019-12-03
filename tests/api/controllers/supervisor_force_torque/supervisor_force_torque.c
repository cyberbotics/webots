#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/position_sensor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32


int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int i = 0;
  WbNodeRef node[4];
  WbDeviceTag ps[4];

  node[0] = wb_supervisor_node_get_from_def("BALANCE_0");
  node[1] = wb_supervisor_node_get_from_def("BALANCE_1");
  node[2] = wb_supervisor_node_get_from_def("BALANCE_2");
  node[3] = wb_supervisor_node_get_from_def("BALANCE_3");

  ps[0] = wb_robot_get_device("ps0");
  ps[1] = wb_robot_get_device("ps1");
  ps[2] = wb_robot_get_device("ps2");
  ps[3] = wb_robot_get_device("ps3");
  
  wb_position_sensor_enable(ps[0], TIME_STEP);
  wb_position_sensor_enable(ps[1], TIME_STEP);
  wb_position_sensor_enable(ps[2], TIME_STEP);
  wb_position_sensor_enable(ps[3], TIME_STEP);

  for (i = 0; i < 20; ++i)
    wb_robot_step(TIME_STEP);
    
  for (i = 0; i < 4; ++i)
    ts_assert_double_in_delta(wb_position_sensor_get_value(ps[0]), 0.0, 0.01, "All position angle should be ~0.0 before applying any force/torque");

  const double force[] = {0.0, 1.0, 0.0};
  const double torque[] = {10.0, 10.0, 10.0};
  const double offset0[] = {0.0, 0.0, 0.15};
  const double offset1[] = {0.0, 0.0, -0.15};
  const double offset2[] = {0.0, 0.0, 0.0};

  for (i = 0; i < 20; ++i) {
    wb_robot_step(TIME_STEP);
    wb_supervisor_node_add_relative_force(node[0], force, offset0);
    wb_supervisor_node_add_relative_force(node[1], force, offset1);
    wb_supervisor_node_add_relative_force(node[2], force, offset2);
    wb_supervisor_node_add_torque(node[3], torque);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
