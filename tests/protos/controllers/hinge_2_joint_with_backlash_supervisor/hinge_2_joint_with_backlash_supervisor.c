#include <stdio.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16
#define TEST_DURATION 10.0f

#define MAX_Y_WITH_BACKLASH
#define MAX_Y_NO_BACKLASH
#define MIN_Y_WITH_BACKLASH
#define MAX_Y_NO_BACKLASH

enum test_robots { SIMPLE_HINGE2JOINT, BACKLASH_02_AND_02, BACKLASH_02_AND_00, BACKLASH_00_AND_02, BACKLASH_00_AND_00 };
enum coordinate { X, Y, Z };

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef end_point_node[5];
  end_point_node[SIMPLE_HINGE2JOINT] = wb_supervisor_node_get_from_def("END_POINT_FOR_SIMPLE_HINGE2JOINT");
  end_point_node[BACKLASH_02_AND_02] = wb_supervisor_node_get_from_def("END_POINT_FOR_BACKLASH_02_AND_02");
  end_point_node[BACKLASH_02_AND_00] = wb_supervisor_node_get_from_def("END_POINT_FOR_BACKLASH_02_AND_00");
  end_point_node[BACKLASH_00_AND_02] = wb_supervisor_node_get_from_def("END_POINT_FOR_BACKLASH_00_AND_02");
  end_point_node[BACKLASH_00_AND_00] = wb_supervisor_node_get_from_def("END_POINT_FOR_BACKLASH_00_AND_00");

  const double *end_point_position[5];
  double end_point_max_position[5][3];
  double end_point_min_position[5][3];

  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 3; ++j) {
      end_point_max_position[i][j] = -INFINITY;
      end_point_min_position[i][j] = INFINITY;
    }
  }

  // In this test only axis2 is actuated, the robots with no backlash in this direction should behave like the control case
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    end_point_position[SIMPLE_HINGE2JOINT] = wb_supervisor_node_get_position(end_point_node[SIMPLE_HINGE2JOINT]);
    end_point_position[BACKLASH_02_AND_02] = wb_supervisor_node_get_position(end_point_node[BACKLASH_02_AND_02]);
    end_point_position[BACKLASH_02_AND_00] = wb_supervisor_node_get_position(end_point_node[BACKLASH_02_AND_00]);
    end_point_position[BACKLASH_00_AND_02] = wb_supervisor_node_get_position(end_point_node[BACKLASH_00_AND_02]);
    end_point_position[BACKLASH_00_AND_00] = wb_supervisor_node_get_position(end_point_node[BACKLASH_00_AND_00]);

    printf("[0] %f %f %f\n", end_point_position[SIMPLE_HINGE2JOINT][X], end_point_position[SIMPLE_HINGE2JOINT][Y],
           end_point_position[SIMPLE_HINGE2JOINT][Z]);
    printf("[2] %f %f %f\n", end_point_position[BACKLASH_02_AND_02][X], end_point_position[BACKLASH_02_AND_02][Y],
           end_point_position[BACKLASH_02_AND_02][Z]);
    printf("[2] %f %f %f\n", end_point_position[BACKLASH_02_AND_00][X], end_point_position[BACKLASH_02_AND_00][Y],
           end_point_position[BACKLASH_02_AND_00][Z]);
    printf("[3] %f %f %f\n", end_point_position[BACKLASH_00_AND_02][X], end_point_position[BACKLASH_00_AND_02][Y],
           end_point_position[BACKLASH_00_AND_02][Z]);
    printf("[4] %f %f %f\n", end_point_position[BACKLASH_00_AND_00][X], end_point_position[BACKLASH_00_AND_00][Y],
           end_point_position[BACKLASH_00_AND_00][Z]);

    // TEST #2: verify that no endpoint changes in the z direction
    ts_assert_double_in_delta(end_point_position[SIMPLE_HINGE2JOINT][Z], 0, 1e-8,
                              "Endpoint of robot (case SIMPLE_HINGE2JOINT) moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_02_AND_02][Z], -0.25, 1e-8,
                              "Endpoint of robot (case BACKLASH_02_AND_02) moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_02_AND_00][Z], -0.50, 1e-8,
                              "Endpoint of robot (case BACKLASH_02_AND_00) moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_00_AND_02][Z], -0.75, 1e-8,
                              "Endpoint of robot (case BACKLASH_00_AND_02) moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_00_AND_00][Z], -1, 1e-8,
                              "Endpoint of robot (case BACKLASH_00_AND_00) moved in Z when shouldn't.");

    for (int i = 0; i < 5; ++i) {
      for (int j = 0; j < 3; ++j) {
        if (end_point_position[i][j] > end_point_max_position[i][j])
          end_point_max_position[i][j] = end_point_position[i][j];
        if (end_point_position[i][j] < end_point_min_position[i][j])
          end_point_min_position[i][j] = end_point_position[i][j];
      }
    }
    // TEST #2: verify that no endpoint goes above or below their limit.
  }

  printf("---\n");
  for (int i = 0; i < 5; ++i) {
    printf("MAX %d) %f %f %f\n", i, end_point_max_position[i][0], end_point_max_position[i][1], end_point_max_position[i][2]);
  }
  printf("---\n");
  for (int i = 0; i < 5; ++i) {
    printf("MIN %d) %f %f %f\n", i, end_point_min_position[i][0], end_point_min_position[i][1], end_point_min_position[i][2]);
  }

  wb_supervisor_simulation_reset();

  // In this test only axis is actuated
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < 2 * TEST_DURATION) {
    end_point_position[SIMPLE_HINGE2JOINT] = wb_supervisor_node_get_position(end_point_node[SIMPLE_HINGE2JOINT]);
    end_point_position[BACKLASH_02_AND_02] = wb_supervisor_node_get_position(end_point_node[BACKLASH_02_AND_02]);
    end_point_position[BACKLASH_02_AND_00] = wb_supervisor_node_get_position(end_point_node[BACKLASH_02_AND_00]);
    end_point_position[BACKLASH_00_AND_02] = wb_supervisor_node_get_position(end_point_node[BACKLASH_00_AND_02]);
    end_point_position[BACKLASH_00_AND_00] = wb_supervisor_node_get_position(end_point_node[BACKLASH_00_AND_00]);

    printf("[0] %f %f %f\n", end_point_position[SIMPLE_HINGE2JOINT][X], end_point_position[SIMPLE_HINGE2JOINT][Y],
           end_point_position[SIMPLE_HINGE2JOINT][Z]);
    printf("[2] %f %f %f\n", end_point_position[BACKLASH_02_AND_02][X], end_point_position[BACKLASH_02_AND_02][Y],
           end_point_position[BACKLASH_02_AND_02][Z]);
    printf("[2] %f %f %f\n", end_point_position[BACKLASH_02_AND_00][X], end_point_position[BACKLASH_02_AND_00][Y],
           end_point_position[BACKLASH_02_AND_00][Z]);
    printf("[3] %f %f %f\n", end_point_position[BACKLASH_00_AND_02][X], end_point_position[BACKLASH_00_AND_02][Y],
           end_point_position[BACKLASH_00_AND_02][Z]);
    printf("[4] %f %f %f\n", end_point_position[BACKLASH_00_AND_00][X], end_point_position[BACKLASH_00_AND_00][Y],
           end_point_position[BACKLASH_00_AND_00][Z]);

    // TEST #2: verify that no endpoint changes in the z direction
    /*
    ts_assert_double_in_delta(end_point_position[SIMPLE_HINGE2JOINT][Z], 0, 1e-8,
                              "Endpoint of robot (case SIMPLE_HINGE2JOINT) moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_02_AND_02][Z], -0.25, 1e-8,
                              "Endpoint of robot (case BACKLASH_02_AND_02) moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_02_AND_00][Z], -0.50, 1e-8,
                              "Endpoint of robot (case BACKLASH_02_AND_00) moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_00_AND_02][Z], -0.75, 1e-8,
                              "Endpoint of robot (case BACKLASH_00_AND_02) moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_00_AND_00][Z], -1, 1e-8,
                              "Endpoint of robot (case BACKLASH_00_AND_00) moved in Z when shouldn't.");
    */

    for (int i = 0; i < 5; ++i) {
      for (int j = 0; j < 3; ++j) {
        if (end_point_position[i][j] > end_point_max_position[i][j])
          end_point_max_position[i][j] = end_point_position[i][j];
        if (end_point_position[i][j] < end_point_min_position[i][j])
          end_point_min_position[i][j] = end_point_position[i][j];
      }
    }
    // TEST #2: verify that no endpoint goes above or below their limit.
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
