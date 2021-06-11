#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, char **argv) {
  ts_setup(argv[0]);  // give the controller args
  
  /*
  
  WbNodeRef parent_node = wb_supervisor_node_get_from_def("PARENT");
  WbFieldRef translation = wb_supervisor_node_get_field(parent_node, "translation");

  // Test position update of node defined in nested PROTO parameter
  WbNodeRef parameter_node = wb_supervisor_node_get_from_def("PARAMETER");
  const double *parameter_position = wb_supervisor_node_get_position(parameter_node);
  ts_assert_vec3_in_delta(parameter_position[0], parameter_position[1], parameter_position[2], 0.45, 0.5, 0.03, 0.0001,
                          "Position of visible nested PROTO parameter is wrong.");

  // Test position update of node defined in a PROTO definition
  WbNodeRef test_node = wb_supervisor_node_get_from_proto_def(parent_node, "TEST_PROTO");
  wb_robot_step(TIME_STEP);
  const double *position_before = wb_supervisor_node_get_position(test_node);
  printf("%10f %10f %10f\n", position_before[0], position_before[1], position_before[2]);

  ts_assert_vec3_in_delta(position_before[0], position_before[1], position_before[2], 3.0, 0.0, 1.0, 0.0001,
                          "Initial position of deeply nested PROTO parameter is wrong.");

 

  const double new_position[3] = {0.0, 1.0, 0.0};
  wb_supervisor_field_set_sf_vec3f(translation, new_position);

  wb_robot_step(TIME_STEP);

  const double *position_after = wb_supervisor_node_get_position(test_node);
  ts_assert_vec3_in_delta(position_after[0], position_after[1], position_after[2], 2.0, 1.0, 1.0, 0.0001,
                         "Position of deeply nested PROTO parameter is wrong after top node position change.");
  
  wb_supervisor_simulation_reset();
  wb_robot_step(TIME_STEP);

  position_before = wb_supervisor_node_get_position(test_node);
  printf("%10f %10f %10f\n", position_before[0], position_before[1], position_before[2]);

  ts_assert_vec3_in_delta(position_before[0], position_before[1], position_before[2], 3.0, 0.0, 1.0, 0.0001,
                          "Initial position of deeply nested PROTO parameter is wrong.");
  */
  
  
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
  
  /*
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
  */
  wb_robot_step(100*TIME_STEP);
  
  
  ts_send_success();
  return EXIT_SUCCESS;
}
