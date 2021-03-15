#include <stdio.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16
#define TEST_DURATION 10.0f
#define VELOCITY 0.2f

enum { BACKLASH_ON_BOTH, BACKLASH_ON_AXIS, BACKLASH_ON_AXIS2, BACKLASH_ON_NEITHER, REFERENCE_HINGE2JOINT };
enum coordinate { X, Y, Z };

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");
  WbDeviceTag emitter = wb_robot_get_device("emitter");

  WbNodeRef world_info_node = wb_supervisor_node_get_from_def("WORLD_INFO");
  WbFieldRef gravity_field = wb_supervisor_node_get_field(world_info_node, "gravity");
  wb_supervisor_field_set_sf_float(gravity_field, 0);

  WbNodeRef end_point_node[5];
  end_point_node[BACKLASH_ON_BOTH] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_BOTH");
  end_point_node[BACKLASH_ON_AXIS] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_AXIS");
  end_point_node[BACKLASH_ON_AXIS2] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_AXIS2");
  end_point_node[BACKLASH_ON_NEITHER] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_NEITHER");
  end_point_node[REFERENCE_HINGE2JOINT] = wb_supervisor_node_get_from_def("END_POINT_CASE_REFERENCE_HINGE2JOINT");
  const double *end_point_position[5];

  double reference_min_y = INFINITY;
  double reference_max_y = -INFINITY;

  // Test: in this test only axis2 is actuated, the joints with no backlash in this direction should behave
  float outbuffer[2] = {0.0f, VELOCITY};
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    // send command
    wb_emitter_send(emitter, outbuffer, 2 * sizeof(float));
    // monitoring
    end_point_position[BACKLASH_ON_BOTH] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_BOTH]);
    end_point_position[BACKLASH_ON_AXIS] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_AXIS]);
    end_point_position[BACKLASH_ON_AXIS2] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_AXIS2]);
    end_point_position[BACKLASH_ON_NEITHER] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_NEITHER]);
    end_point_position[REFERENCE_HINGE2JOINT] = wb_supervisor_node_get_position(end_point_node[REFERENCE_HINGE2JOINT]);

    printf("[0] %.10f %.10f %.10f\n", end_point_position[BACKLASH_ON_BOTH][X], end_point_position[BACKLASH_ON_BOTH][Y],
           end_point_position[BACKLASH_ON_BOTH][Z]);
    printf("[1] %.10f %.10f %.10f\n", end_point_position[BACKLASH_ON_AXIS][X], end_point_position[BACKLASH_ON_AXIS][Y],
           end_point_position[BACKLASH_ON_AXIS][Z]);
    printf("[2] %.10f %.10f %.10f\n", end_point_position[BACKLASH_ON_AXIS2][X], end_point_position[BACKLASH_ON_AXIS2][Y],
           end_point_position[BACKLASH_ON_AXIS2][Z]);
    printf("[3] %.10f %.10f %.10f\n", end_point_position[BACKLASH_ON_NEITHER][X], end_point_position[BACKLASH_ON_NEITHER][Y],
           end_point_position[BACKLASH_ON_NEITHER][Z]);
    printf("[4] %.10f %.10f %.10f\n", end_point_position[REFERENCE_HINGE2JOINT][X],
           end_point_position[REFERENCE_HINGE2JOINT][Y], end_point_position[REFERENCE_HINGE2JOINT][Z]);
    printf("----\n");
    // TEST: verify that no endpoint changes in the z direction
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_BOTH][Z], 0, 1e-7,
                              "In test case 'BACKLASH_ON_BOTH' the endpoint moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_AXIS][Z], -0.25, 1e-7,
                              "In test case 'BACKLASH_ON_AXIS' the endpoint moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_AXIS2][Z], -0.50, 1e-7,
                              "In test case 'BACKLASH_ON_AXIS2 the endpoint moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_NEITHER][Z], -0.75, 1e-7,
                              "In test case 'BACKLASH_ON_NEITHER' the endpoint moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[REFERENCE_HINGE2JOINT][Z], -1, 1e-7,
                              "In test case 'REFERENCE_HINGE2JOINT' the endpoint moved in Z when shouldn't.");

    // keep track of reference position y (minimum and maximum)
    if (end_point_position[REFERENCE_HINGE2JOINT][Y] > reference_max_y)
      reference_max_y = end_point_position[REFERENCE_HINGE2JOINT][Y];
    if (end_point_position[REFERENCE_HINGE2JOINT][Y] < reference_min_y)
      reference_min_y = end_point_position[REFERENCE_HINGE2JOINT][Y];

    // TEST: no endpoint should swing beyond the highest point of the reference.
    const double tolerance = 1e-8;
    // check against highest point
    ts_assert_double_is_bigger(reference_max_y + tolerance, end_point_position[BACKLASH_ON_BOTH][Y],
                               "In test case 'BACKLASH_ON_BOTH' the endpoint moved above the reference.");
    ts_assert_double_is_bigger(reference_max_y + tolerance, end_point_position[BACKLASH_ON_AXIS][Y],
                               "In test case 'BACKLASH_ON_AXIS' the endpoint moved above the reference.");
    ts_assert_double_is_bigger(reference_max_y + tolerance, end_point_position[BACKLASH_ON_AXIS2][Y],
                               "In test case 'BACKLASH_ON_AXIS2' the endpoint moved above the reference.");
    ts_assert_double_is_bigger(reference_max_y + tolerance, end_point_position[BACKLASH_ON_NEITHER][Y],
                               "In test case 'BACKLASH_ON_NEITHER' the endpoint moved above the reference.");
    // check against lowest point
    if (wb_robot_get_time() > 0.6) {  // only makes sense to check this after the ones with backlash begin to move
      ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_BOTH][Y], reference_min_y - tolerance,
                                 "In test case 'BACKLASH_ON_BOTH' the endpoint moved below the reference.");
      ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_AXIS][Y], reference_min_y - tolerance,
                                 "In test case 'BACKLASH_ON_AXIS' the endpoint moved below the reference.");
      ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_AXIS2][Y], reference_min_y - tolerance,
                                 "In test case 'BACKLASH_ON_AXIS2' the endpoint moved below the reference.");
      ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_NEITHER][Y], reference_min_y - tolerance,
                                 "In test case 'BACKLASH_ON_NEITHER' the endpoint moved below the reference.");
    }
    // TEST: assert if behavior is consistent (those with backlash behave all the same, those without as well)
    // (BACKLASH_ON_BOTH, BACKLASH_ON_AXIS2) should behave in the same manner
    ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_BOTH][X], end_point_position[BACKLASH_ON_BOTH][Y], 0,
                            end_point_position[BACKLASH_ON_AXIS2][X], end_point_position[BACKLASH_ON_AXIS2][Y], 0, 1e-8,
                            "'BACKLASH_ON_BOTH' and 'BACKLASH_ON_AXIS2' not behaving in the same manner.");
    // (BACKLASH_ON_AXIS, BACKLASH_ON_NEITHER, REFERENCE_HINGE2JOINT) should behave in the same manner
    ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_AXIS][X], end_point_position[BACKLASH_ON_AXIS][Y], 0,
                            end_point_position[REFERENCE_HINGE2JOINT][X], end_point_position[REFERENCE_HINGE2JOINT][Y], 0, 1e-8,
                            "'BACKLASH_ON_AXIS' and 'REFERENCE_HINGE2JOINT' not behaving in the same manner.");
    ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_NEITHER][X], end_point_position[BACKLASH_ON_NEITHER][Y], 0,
                            end_point_position[REFERENCE_HINGE2JOINT][X], end_point_position[REFERENCE_HINGE2JOINT][Y], 0, 1e-8,
                            "'BACKLASH_ON_NEITHER' and 'REFERENCE_HINGE2JOINT' not behaving in the same manner.");
    // the first group should behave differently from the second
    if (wb_robot_get_time() > TIME_STEP && wb_robot_get_time() < 2.5) {  // sufficient to test that they start differently
      ts_assert_double_is_bigger(fabs(end_point_position[BACKLASH_ON_BOTH][Y] - end_point_position[REFERENCE_HINGE2JOINT][Y]),
                                 1e-5, "'BACKLASH_ON_BOTH' and 'REFERENCE_HINGE2JOINT' behave similarly but shouldn't.");
    }
  }

  wb_supervisor_simulation_reset();
  wb_supervisor_node_restart_controller(robot);

  wb_robot_step(TIME_STEP);

  wb_supervisor_field_set_sf_float(gravity_field, 9.81);

  double reference_min_z = INFINITY;
  double reference_max_z = -INFINITY;

  // Test: in this test only axis is actuated, the joints with no backlash in this direction
  outbuffer[0] = VELOCITY;
  outbuffer[1] = 0.0f;
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    // send command
    wb_emitter_send(emitter, outbuffer, 2 * sizeof(float));
    // monitoring
    end_point_position[BACKLASH_ON_BOTH] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_BOTH]);
    end_point_position[BACKLASH_ON_AXIS] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_AXIS]);
    end_point_position[BACKLASH_ON_AXIS2] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_AXIS2]);
    end_point_position[BACKLASH_ON_NEITHER] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_NEITHER]);
    end_point_position[REFERENCE_HINGE2JOINT] = wb_supervisor_node_get_position(end_point_node[REFERENCE_HINGE2JOINT]);

    printf("[0] %.10f %.10f %.10f\n", end_point_position[BACKLASH_ON_BOTH][X], end_point_position[BACKLASH_ON_BOTH][Y],
           end_point_position[BACKLASH_ON_BOTH][Z]);
    printf("[1] %.10f %.10f %.10f\n", end_point_position[BACKLASH_ON_AXIS][X], end_point_position[BACKLASH_ON_AXIS][Y],
           end_point_position[BACKLASH_ON_AXIS][Z] + 0.25);
    printf("[2] %.10f %.10f %.10f\n", end_point_position[BACKLASH_ON_AXIS2][X], end_point_position[BACKLASH_ON_AXIS2][Y],
           end_point_position[BACKLASH_ON_AXIS2][Z] + 0.5);
    printf("[3] %.10f %.10f %.10f\n", end_point_position[BACKLASH_ON_NEITHER][X], end_point_position[BACKLASH_ON_NEITHER][Y],
           end_point_position[BACKLASH_ON_NEITHER][Z] + 0.75);
    printf("[4] %.10f %.10f %.10f\n", end_point_position[REFERENCE_HINGE2JOINT][X],
           end_point_position[REFERENCE_HINGE2JOINT][Y], end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0);
    printf("----\n");
    // TEST: verify that no endpoint changes in the y direction
    // note: BACKLASH_ON_BOTH and BACKLASH_ON_AXIS2 have different targets as they aren't constrained in Y due to backlash so
    // they settle further down
    if (wb_robot_get_time() > 1.0) {  // wait for gravity to bring it down
      ts_assert_double_in_delta(end_point_position[BACKLASH_ON_BOTH][Y], -0.200999, 1e-6,
                                "In test case 'BACKLASH_ON_BOTH' the endpoint moved in Z when shouldn't.");
      ts_assert_double_in_delta(end_point_position[BACKLASH_ON_AXIS][Y], -0.200001, 1e-6,
                                "In test case 'BACKLASH_ON_AXIS' the endpoint moved in Z when shouldn't.");
      ts_assert_double_in_delta(end_point_position[BACKLASH_ON_AXIS2][Y], -0.200999, 1e-6,
                                "In test case 'BACKLASH_ON_AXIS2 the endpoint moved in Z when shouldn't.");
      ts_assert_double_in_delta(end_point_position[BACKLASH_ON_NEITHER][Y], -0.200001, 1e-6,
                                "In test case 'BACKLASH_ON_NEITHER' the endpoint moved in Z when shouldn't.");
      ts_assert_double_in_delta(end_point_position[REFERENCE_HINGE2JOINT][Y], -0.200001, 1e-6,
                                "In test case 'REFERENCE_HINGE2JOINT' the endpoint moved in Z when shouldn't.");
    }

    // keep track of reference position z (minimum and maximum)
    if (end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0 > reference_max_z)
      reference_max_z = end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0;
    if (end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0 < reference_min_z)
      reference_min_z = end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0;

    // TEST: no endpoint should swing beyond the limits of the reference.
    const double eps = 1e-5;  // due to stop penetration some leeway is necessary
    // check against highest point
    ts_assert_double_is_bigger(reference_max_z + eps, end_point_position[BACKLASH_ON_BOTH][Z] + 0.0,
                               "In test case 'BACKLASH_ON_BOTH' the endpoint moved in front of the reference.");
    ts_assert_double_is_bigger(reference_max_z + eps, end_point_position[BACKLASH_ON_AXIS][Z] + 0.25,
                               "In test case 'BACKLASH_ON_AXIS' the endpoint moved in front of the reference.");
    ts_assert_double_is_bigger(reference_max_z + eps, end_point_position[BACKLASH_ON_AXIS2][Z] + 0.50,
                               "In test case 'BACKLASH_ON_AXIS2' the endpoint moved in front of the reference.");
    ts_assert_double_is_bigger(reference_max_z + eps, end_point_position[BACKLASH_ON_NEITHER][Z] + 0.75,
                               "In test case 'BACKLASH_ON_NEITHER' the endpoint moved in front of the reference.");
  }
  /*
  // check against lowest point
  if (wb_robot_get_time() > 0.6) {  // only makes sense to check this after the ones with backlash begin to move
    // ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_BOTH][Z], reference_min_z - eps,
    //      "In test case 'BACKLASH_ON_BOTH' the endpoint moved below the reference.");
    ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_AXIS][Z] + 0.25, reference_min_z - eps,
                               "In test case 'BACKLASH_ON_AXIS' the endpoint moved below the reference.");
    // ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_AXIS2][Z], reference_min_z - eps,
    //      "In test case 'BACKLASH_ON_AXIS2' the endpoint moved below the reference.");
    ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_NEITHER][Z] + 0.75, reference_min_z - eps,
                               "In test case 'BACKLASH_ON_NEITHER' the endpoint moved below the reference.");
  }

  printf("%f %f / %f %f\n", end_point_position[BACKLASH_ON_AXIS2][X], end_point_position[REFERENCE_HINGE2JOINT][X],
         end_point_position[BACKLASH_ON_AXIS2][Y], end_point_position[REFERENCE_HINGE2JOINT][Y]);
  // TEST: assert if behavior is consistent (those with backlash behave all the same, those without as well)
  // (BACKLASH_ON_BOTH, BACKLASH_ON_AXIS) should behave in the same manner
  ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_BOTH][X], end_point_position[BACKLASH_ON_BOTH][Y], 0,
                          end_point_position[BACKLASH_ON_AXIS][X], end_point_position[BACKLASH_ON_AXIS][Y], 0, 1e-3,
                          "'BACKLASH_ON_BOTH' and 'BACKLASH_ON_AXIS' not behaving in the same manner.");
  // (BACKLASH_ON_AXIS2, BACKLASH_ON_NEITHER, REFERENCE_HINGE2JOINT) should behave in the same manner
  ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_AXIS2][X], end_point_position[BACKLASH_ON_AXIS2][Y], 0,
                          end_point_position[REFERENCE_HINGE2JOINT][X], end_point_position[REFERENCE_HINGE2JOINT][Y], 0, 2e-3,
                          "'BACKLASH_ON_AXIS2' and 'REFERENCE_HINGE2JOINT' not behaving in the same manner.");
  ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_NEITHER][X], end_point_position[BACKLASH_ON_NEITHER][Y], 0,
                          end_point_position[REFERENCE_HINGE2JOINT][X], end_point_position[REFERENCE_HINGE2JOINT][Y], 0, 1e-3,
                          "'BACKLASH_ON_NEITHER' and 'REFERENCE_HINGE2JOINT' not behaving in the same manner.");
  // the first group should behave differently from the second

      if (wb_robot_get_time() < 2.5) {  // sufficient to test that they start differently
        ts_assert_double_is_bigger(fabs(end_point_position[BACKLASH_ON_BOTH][Z] -
     end_point_position[REFERENCE_HINGE2JOINT][Z]), 1e-5, "'BACKLASH_ON_BOTH' and 'REFERENCE_HINGE2JOINT' behave similarly but
     shouldn't.");
      }

}
*/
  ts_send_success();
  return EXIT_SUCCESS;
}
