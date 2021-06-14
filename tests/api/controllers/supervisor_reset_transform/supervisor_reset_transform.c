/*
 * Description:  Test Supervisor device API
 *               This file contains tests of reset simulation method
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define TOLERANCE 1E-8

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef outer_nodes[3];
  char outer_node_names[3][15] = {"OUTER_T1_NODE", "OUTER_T2_NODE", "OUTER_T3_NODE"};
  WbFieldRef outer_nodes_translations[6];

  WbNodeRef inner_nodes[3];
  char inner_node_names[3][15] = {"INNER_T1_NODE", "INNER_S2_NODE", "INNER_S3_NODE"};
  // get node and field references
  for (int i = 0; i < 3; ++i) {
    outer_nodes[i] = wb_supervisor_node_get_from_def(outer_node_names[i]);
    outer_nodes_translations[i] = wb_supervisor_node_get_field(outer_nodes[i], "translation");
    inner_nodes[i] = wb_supervisor_node_get_from_def(inner_node_names[i]);
  }

  // get initial positions
  double expected_initial_outer[3][3] = {{0, 0, 0}, {2, 0, 0}, {4, 0, 0}};
  double expected_initial_inner[3][3] = {{0, 0, 1}, {2, 0, 1}, {4, 0, 1}};

  const double *outer_initial_positions[3];
  const double *inner_initial_positions[3];
  for (int i = 0; i < 3; ++i) {
    outer_initial_positions[i] = wb_supervisor_node_get_position(outer_nodes[i]);
    inner_initial_positions[i] = wb_supervisor_node_get_position(inner_nodes[i]);

    ts_assert_doubles_in_delta(3, outer_initial_positions[i], expected_initial_outer[i], TOLERANCE,
                               "Test 1: 'outer' position for case %d is incorrect.", i);
    ts_assert_doubles_in_delta(3, inner_initial_positions[i], expected_initial_inner[i], TOLERANCE,
                               "Test 1: 'inner' position for case %d is incorrect.", i);
  }

  // move outer node by (0 0 5)
  double new_translation[3][3] = {{0, 0, 5}, {2, 0, 5}, {4, 0, 5}};
  for (int i = 0; i < 3; ++i)
    wb_supervisor_field_set_sf_vec3f(outer_nodes_translations[i], new_translation[i]);

  wb_robot_step(TIME_STEP);

  // retrieve position again and verify both inner and outer moved
  double expected_later_outer[3][3] = {{0, 0, 5}, {2, 0, 5}, {4, 0, 5}};
  double expected_later_inner[3][3] = {{0, 0, 6}, {2, 0, 6}, {4, 0, 6}};
  const double *outer_later_positions[3];
  const double *inner_later_positions[3];
  for (int i = 0; i < 3; ++i) {
    outer_later_positions[i] = wb_supervisor_node_get_position(outer_nodes[i]);
    inner_later_positions[i] = wb_supervisor_node_get_position(inner_nodes[i]);

    ts_assert_doubles_in_delta(3, outer_later_positions[i], expected_later_outer[i], TOLERANCE,
                               "Test 2: 'outer' position is incorrect for case %d.", i);
    ts_assert_doubles_in_delta(3, inner_later_positions[i], expected_later_inner[i], TOLERANCE,
                               "Test 2: 'inner' position is incorrect for case %d.", i);
  }

  // reset
  wb_supervisor_simulation_reset();
  wb_robot_step(TIME_STEP);

  // retrieve inner and outer positions again
  const double *outer_reset_positions[3];
  const double *inner_reset_positions[3];
  for (int i = 0; i < 3; ++i) {
    outer_reset_positions[i] = wb_supervisor_node_get_position(outer_nodes[i]);
    inner_reset_positions[i] = wb_supervisor_node_get_position(inner_nodes[i]);

    ts_assert_doubles_in_delta(3, outer_reset_positions[i], expected_initial_outer[i], TOLERANCE,
                               "Test 3: 'outer' position does not coincide with initial one for case %d.", i);
    ts_assert_doubles_in_delta(3, inner_reset_positions[i], expected_initial_inner[i], TOLERANCE,
                               "Test 3: 'inner' position position does not coincide with initial one for case %d.", i);
  }

  // ensure current position coincides with initial position
  for (int i = 0; i < 3; ++i) {
    ts_assert_doubles_in_delta(3, outer_reset_positions[i], outer_initial_positions[i], TOLERANCE,
                               "Test 4: 'outer' node %d did not return to initial value after reset.", i);
    ts_assert_doubles_in_delta(3, inner_reset_positions[i], inner_initial_positions[i], TOLERANCE,
                               "Test 4: 'inner' node %d did not return to initial value after reset.", i);
  }

  // move outer node by (0 0 5) again
  for (int i = 0; i < 3; ++i)
    wb_supervisor_field_set_sf_vec3f(outer_nodes_translations[i], new_translation[i]);

  wb_robot_step(TIME_STEP);

  const double *outer_reset_later_positions[3];
  const double *inner_reset_later_positions[3];
  for (int i = 0; i < 3; ++i) {
    outer_reset_later_positions[i] = wb_supervisor_node_get_position(outer_nodes[i]);
    inner_reset_later_positions[i] = wb_supervisor_node_get_position(inner_nodes[i]);

    ts_assert_doubles_in_delta(3, outer_reset_later_positions[i], expected_later_outer[i], TOLERANCE,
                               "After move and after reset, 'outer' position is incorrect for case %d.", i);
    ts_assert_doubles_in_delta(3, inner_reset_later_positions[i], expected_later_inner[i], TOLERANCE,
                               "After move and after reset, 'inner' position is incorrect for case %d.", i);
  }

  // save current state
  for (int i = 0; i < 3; ++i)
    wb_supervisor_node_save_state(outer_nodes[i], "intermediary_state");

  // move outer node by an additional (0 0 5)
  double newer_translation[3][3] = {{0, 0, 10}, {2, 0, 10}, {4, 0, 10}};
  for (int i = 0; i < 3; ++i)
    wb_supervisor_field_set_sf_vec3f(outer_nodes_translations[i], newer_translation[i]);

  wb_robot_step(TIME_STEP);

  // retrieve position again and verify both inner and outer moved
  double expected_later_outer[3][3] = {{0, 0, 10}, {2, 0, 10}, {4, 0, 10}};
  double expected_later_inner[3][3] = {{0, 0, 11}, {2, 0, 11}, {4, 0, 11}};
  const double *outer_later_positions[3];
  const double *inner_later_positions[3];
  for (int i = 0; i < 3; ++i) {
    outer_later_positions[i] = wb_supervisor_node_get_position(outer_nodes[i]);
    inner_later_positions[i] = wb_supervisor_node_get_position(inner_nodes[i]);

    ts_assert_doubles_in_delta(3, outer_later_positions[i], expected_later_outer[i], TOLERANCE,
                               "After move, 'outer' position is incorrect for case %d.", i);
    ts_assert_doubles_in_delta(3, inner_later_positions[i], expected_later_inner[i], TOLERANCE,
                               "After move, 'inner' position is incorrect for case %d.", i);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
