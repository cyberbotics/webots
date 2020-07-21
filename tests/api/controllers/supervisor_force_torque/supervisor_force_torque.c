#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define SOLID_NUMBER 9

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int i = 0;
  char buffer[32];
  WbNodeRef node[SOLID_NUMBER];
  WbDeviceTag ps[SOLID_NUMBER];

  for (i = 0; i < SOLID_NUMBER; ++i) {
    sprintf(buffer, "BALANCE_%d", i);
    node[i] = wb_supervisor_node_get_from_def(buffer);
    sprintf(buffer, "ps%d", i);
    ps[i] = wb_robot_get_device(buffer);
    wb_position_sensor_enable(ps[i], TIME_STEP);
  }

  for (i = 0; i < 20; ++i)
    wb_robot_step(TIME_STEP);

  for (i = 0; i < SOLID_NUMBER; ++i)
    ts_assert_double_in_delta(wb_position_sensor_get_value(ps[i]), 0.0, 0.01,
                              "All position angle should be ~0.0 before applying any force/torque");

  const double force0[] = {0.0, 0.1, 0.0};
  const double force1[] = {0.1, 0.0, 0.0};
  const double torque0[] = {0.1, 0.0, 0.0};
  const double torque1[] = {0.0, 0.1, 0.0};
  const double offset0[] = {0.0, 0.0, 0.15};
  const double offset1[] = {0.0, 0.0, -0.15};
  const double offset2[] = {0.0, 0.0, 0.0};

  for (i = 0; i < 200; ++i) {
    wb_robot_step(TIME_STEP);
    wb_supervisor_node_add_force_with_offset(node[0], force0, offset0, false);
    wb_supervisor_node_add_force_with_offset(node[1], force0, offset1, false);
    wb_supervisor_node_add_force_with_offset(node[2], force0, offset2, false);
    wb_supervisor_node_add_torque(node[3], torque0, false);
    wb_supervisor_node_add_force(node[4], force0, false);
    wb_supervisor_node_add_force_with_offset(node[5], force0, offset0, false);
    wb_supervisor_node_add_torque(node[6], torque0, true);
    wb_supervisor_node_add_torque(node[7], torque1, true);
    wb_supervisor_node_add_force(node[8], force1, true);
  }

  double positions[SOLID_NUMBER];
  for (i = 0; i < SOLID_NUMBER; ++i)
    positions[i] = wb_position_sensor_get_value(ps[i]);

  ts_assert_double_in_delta(positions[0], -positions[1], 0.01, "Angle 0 should be equal to -angle 1.");
  ts_assert_double_in_delta(positions[4], positions[5], 0.01, "Angle 4 should be equal to angle 5.");
  ts_assert_double_in_delta(positions[6], positions[3], 0.01, "Angle 3 should be equal to angle 6.");
  ts_assert_double_in_delta(positions[2], 0.0, 0.01, "Angle 2 should be zero");
  ts_assert_double_is_bigger(-0.2, positions[0], "Angle 0 should be smaller than -0.2");
  ts_assert_double_is_bigger(positions[1], 0.2, "Angle 1 should be bigger than 0.2");
  ts_assert_double_is_bigger(positions[3], 0.2, "Angle 3 should be bigger than 0.2");
  ts_assert_double_is_bigger(-0.2, positions[4], "Angle 4 should be smaller than -0.2");
  ts_assert_double_is_bigger(-0.2, positions[5], "Angle 5 should be smaller than -0.2");
  ts_assert_double_in_delta(positions[7], 0.0, 0.01, "Angle 7 should be zero");
  ts_assert_double_in_delta(positions[8], 0.0, 0.01, "Angle 8 should be zero");

  // let the solid go to sleep
  for (i = 0; i < 200; ++i)
    wb_robot_step(TIME_STEP);

  const double force[] = {0.0, -0.1, 0.0};
  const double torque[] = {-0.1, 0.0, 0.0};
  for (i = 0; i < 200; ++i) {
    wb_robot_step(TIME_STEP);
    wb_supervisor_node_add_force_with_offset(node[0], force, offset0, false);
    wb_supervisor_node_add_torque(node[3], torque, false);
    wb_supervisor_node_add_force(node[4], force, false);
  }

  for (i = 0; i < SOLID_NUMBER; ++i)
    positions[i] = wb_position_sensor_get_value(ps[i]);

  ts_assert_double_is_bigger(positions[0], 0.2, "Angle 0 should be bigger than 0.2");
  ts_assert_double_is_bigger(-0.2, positions[3], "Angle 3 should be smaller than -0.2");
  ts_assert_double_is_bigger(positions[4], 0.2, "Angle 4 should be bigger than 0.2");

  ts_send_success();
  return EXIT_SUCCESS;
}
