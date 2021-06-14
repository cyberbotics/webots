/*
 * Description:  Test Supervisor device API
 *               This file contains tests of reset simulation method
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  printf("START\n");
  WbNodeRef parent_node = wb_supervisor_node_get_from_def("PARENT_NODE");
  WbFieldRef parent_translation = wb_supervisor_node_get_field(parent_node, "translation");

  const double *parent_before = wb_supervisor_node_get_position(parent_node);
  printf("PARENT %10f %10f %10f\n", parent_before[0], parent_before[1], parent_before[2]);

  WbNodeRef inner_node = wb_supervisor_node_get_from_def("INNER_NODE");
  const double *inner_before = wb_supervisor_node_get_position(inner_node);
  printf("INNER %10f %10f %10f\n", inner_before[0], inner_before[1], inner_before[2]);

  const double new_position[3] = {0.0, 0.0, 5.0};
  wb_supervisor_field_set_sf_vec3f(parent_translation, new_position);
  wb_robot_step(TIME_STEP);
  printf("MOVE\n");

  const double *parent_after = wb_supervisor_node_get_position(parent_node);
  printf("PARENT %10f %10f %10f\n", parent_after[0], parent_after[1], parent_after[2]);

  const double *inner_after = wb_supervisor_node_get_position(inner_node);
  printf("INNER %10f %10f %10f\n", inner_after[0], inner_after[1], inner_after[2]);
  printf("END\n");

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  wb_supervisor_simulation_reset();

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  printf("RESTART\n");
  const double *parent_before2 = wb_supervisor_node_get_position(parent_node);
  printf("PARENT %10f %10f %10f\n", parent_before2[0], parent_before2[1], parent_before2[2]);

  const double *inner_before2 = wb_supervisor_node_get_position(inner_node);
  printf("INNER %10f %10f %10f\n", inner_before2[0], inner_before2[1], inner_before2[2]);

  wb_supervisor_field_set_sf_vec3f(parent_translation, new_position);
  wb_robot_step(TIME_STEP);
  printf("MOVE\n");

  const double *parent_after2 = wb_supervisor_node_get_position(parent_node);
  printf("PARENT %10f %10f %10f\n", parent_after2[0], parent_after2[1], parent_after2[2]);

  const double *inner_after2 = wb_supervisor_node_get_position(inner_node);
  printf("INNER %10f %10f %10f\n", inner_after2[0], inner_after2[1], inner_after2[2]);
  printf("END\n");

  wb_robot_step(100 * TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
