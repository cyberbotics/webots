/*
 * Description:  Test Supervisor device API
 *               This file contains tests of reset simulation method for transforms
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define TOLERANCE 1E-10

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef outer_nodes[3];
  char outer_node_names[3][15] = {"OUTER_T1_NODE", "OUTER_T2_NODE", "OUTER_T3_NODE"};
  WbFieldRef outer_nodes_translations[3];

  WbNodeRef inner_nodes[3];
  char inner_node_names[3][15] = {"INNER_T1_NODE", "INNER_S2_NODE", "INNER_S3_NODE"};
  // get node and field references
  for (int i = 0; i < 3; ++i) {
    outer_nodes[i] = wb_supervisor_node_get_from_def(outer_node_names[i]);
    outer_nodes_translations[i] = wb_supervisor_node_get_field(outer_nodes[i], "translation");
    inner_nodes[i] = wb_supervisor_node_get_from_def(inner_node_names[i]);
  }

  // get initial positions
  double expected_outer_position_0[3][3] = {{0, 0, 0}, {2, 0, 0}, {4, 0, 0}};
  double expected_inner_position_0[3][3] = {{0, 0, 1}, {2, 0, 1}, {4, 0, 1}};

  const double *outer_position_0[3];
  const double *inner_position_0[3];
  for (int i = 0; i < 3; ++i) {
    outer_position_0[i] = wb_supervisor_node_get_position(outer_nodes[i]);
    inner_position_0[i] = wb_supervisor_node_get_position(inner_nodes[i]);

    ts_assert_doubles_in_delta(3, outer_position_0[i], expected_outer_position_0[i], TOLERANCE,
                               "Test 1: 'outer' position for case %d is incorrect.", i);
    ts_assert_doubles_in_delta(3, inner_position_0[i], expected_inner_position_0[i], TOLERANCE,
                               "Test 1: 'inner' position for case %d is incorrect.", i);
  }

  // move outer node by (0 0 5)
  double short_translation[3][3] = {{0, 0, 5}, {2, 0, 5}, {4, 0, 5}};
  for (int i = 0; i < 3; ++i)
    wb_supervisor_field_set_sf_vec3f(outer_nodes_translations[i], short_translation[i]);

  wb_robot_step(TIME_STEP);

  // retrieve position again and verify both inner and outer moved
  double expected_outer_position_1[3][3] = {{0, 0, 5}, {2, 0, 5}, {4, 0, 5}};
  double expected_inner_position_1[3][3] = {{0, 0, 6}, {2, 0, 6}, {4, 0, 6}};
  const double *outer_position_1[3];
  const double *inner_position_1[3];
  for (int i = 0; i < 3; ++i) {
    outer_position_1[i] = wb_supervisor_node_get_position(outer_nodes[i]);
    inner_position_1[i] = wb_supervisor_node_get_position(inner_nodes[i]);

    ts_assert_doubles_in_delta(3, outer_position_1[i], expected_outer_position_1[i], TOLERANCE,
                               "Test 2: 'outer' position is incorrect for case %d.", i);
    ts_assert_doubles_in_delta(3, inner_position_1[i], expected_inner_position_1[i], TOLERANCE,
                               "Test 2: 'inner' position is incorrect for case %d.", i);
  }

  // reset
  wb_supervisor_simulation_reset();
  wb_robot_step(TIME_STEP);

  // retrieve inner and outer positions again and ensure they coincide with initial positions
  const double *outer_position_2[3];
  const double *inner_position_2[3];
  for (int i = 0; i < 3; ++i) {
    outer_position_2[i] = wb_supervisor_node_get_position(outer_nodes[i]);
    inner_position_2[i] = wb_supervisor_node_get_position(inner_nodes[i]);

    ts_assert_doubles_in_delta(3, outer_position_2[i], expected_outer_position_0[i], TOLERANCE,
                               "Test 3: 'outer' position does not coincide with initial one for case %d.", i);
    ts_assert_doubles_in_delta(3, inner_position_2[i], expected_inner_position_0[i], TOLERANCE,
                               "Test 3: 'inner' position position does not coincide with initial one for case %d.", i);
  }

  // move outer node by (0 0 5) again
  for (int i = 0; i < 3; ++i)
    wb_supervisor_field_set_sf_vec3f(outer_nodes_translations[i], short_translation[i]);

  wb_robot_step(TIME_STEP);

  const double *outer_position_3[3];
  const double *inner_position_3[3];
  for (int i = 0; i < 3; ++i) {
    outer_position_3[i] = wb_supervisor_node_get_position(outer_nodes[i]);
    inner_position_3[i] = wb_supervisor_node_get_position(inner_nodes[i]);

    ts_assert_doubles_in_delta(3, outer_position_3[i], expected_outer_position_1[i], TOLERANCE,
                               "Test 4: 'outer' position is incorrect for case %d.", i);
    ts_assert_doubles_in_delta(3, inner_position_3[i], expected_inner_position_1[i], TOLERANCE,
                               "Test 4: 'inner' position is incorrect for case %d.", i);
  }

  // save current state
  for (int i = 0; i < 3; ++i)
    wb_supervisor_node_save_state(outer_nodes[i], "intermediary_state");

  // move outer node by an additional (0 0 5)
  double long_translation[3][3] = {{0, 0, 10}, {2, 0, 10}, {4, 0, 10}};
  for (int i = 0; i < 3; ++i)
    wb_supervisor_field_set_sf_vec3f(outer_nodes_translations[i], long_translation[i]);

  wb_robot_step(TIME_STEP);

  // retrieve position again and verify both inner and outer moved
  double expected_outer_position_4[3][3] = {{0, 0, 10}, {2, 0, 10}, {4, 0, 10}};
  double expected_inner_position_4[3][3] = {{0, 0, 11}, {2, 0, 11}, {4, 0, 11}};
  const double *outer_position_4[3];
  const double *inner_position_4[3];
  for (int i = 0; i < 3; ++i) {
    outer_position_4[i] = wb_supervisor_node_get_position(outer_nodes[i]);
    inner_position_4[i] = wb_supervisor_node_get_position(inner_nodes[i]);

    ts_assert_doubles_in_delta(3, outer_position_4[i], expected_outer_position_4[i], TOLERANCE,
                               "Test 5: 'outer' position is incorrect for case %d.", i);
    ts_assert_doubles_in_delta(3, inner_position_4[i], expected_inner_position_4[i], TOLERANCE,
                               "Test 5: 'inner' position is incorrect for case %d.", i);
  }

  // load saved state and check positions
  for (int i = 0; i < 3; ++i)
    wb_supervisor_node_load_state(outer_nodes[i], "intermediary_state");

  wb_robot_step(TIME_STEP);

  double expected_outer_position_5[3][3] = {{0, 0, 5}, {2, 0, 5}, {4, 0, 5}};
  double expected_inner_position_5[3][3] = {{0, 0, 6}, {2, 0, 6}, {4, 0, 6}};
  const double *outer_position_5[3];
  const double *inner_position_5[3];
  for (int i = 0; i < 3; ++i) {
    outer_position_5[i] = wb_supervisor_node_get_position(outer_nodes[i]);
    inner_position_5[i] = wb_supervisor_node_get_position(inner_nodes[i]);

    ts_assert_doubles_in_delta(3, outer_position_5[i], expected_outer_position_5[i], TOLERANCE,
                               "Test 6: 'outer' position is incorrect for case %d.", i);
    ts_assert_doubles_in_delta(3, inner_position_5[i], expected_inner_position_5[i], TOLERANCE,
                               "Test 6: 'inner' position is incorrect for case %d.", i);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
