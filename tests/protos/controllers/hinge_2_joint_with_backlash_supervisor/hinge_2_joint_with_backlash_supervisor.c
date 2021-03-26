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

  WbNodeRef end_point_node[5];
  end_point_node[BACKLASH_ON_BOTH] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_BOTH");
  end_point_node[BACKLASH_ON_AXIS] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_AXIS");
  end_point_node[BACKLASH_ON_AXIS2] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_AXIS2");
  end_point_node[BACKLASH_ON_NEITHER] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_NEITHER");
  end_point_node[REFERENCE_HINGE2JOINT] = wb_supervisor_node_get_from_def("END_POINT_CASE_REFERENCE_HINGE2JOINT");
  const double *end_point_position[5];

  double reference_min_z = INFINITY;
  double reference_max_z = -INFINITY;
  double tolerance;

  // Test: in this test only axis2 is actuated, the joints with no backlash in this direction should behave like the
  // reference.
  float outbuffer[2] = {0.0f, VELOCITY};
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    // send command
    wb_emitter_send(emitter, outbuffer, 2 * sizeof(float));
    if (wb_robot_get_time() <= TIME_STEP / 1000)
      continue;  // give controllers time to receive commands and start
    // monitoring
    end_point_position[BACKLASH_ON_BOTH] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_BOTH]);
    end_point_position[BACKLASH_ON_AXIS] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_AXIS]);
    end_point_position[BACKLASH_ON_AXIS2] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_AXIS2]);
    end_point_position[BACKLASH_ON_NEITHER] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_NEITHER]);
    end_point_position[REFERENCE_HINGE2JOINT] = wb_supervisor_node_get_position(end_point_node[REFERENCE_HINGE2JOINT]);

    // Test: verify that no endpoint changes in the X direction
    tolerance = 1e-10;
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_BOTH][X], 0, tolerance,
                              "In test case 'BACKLASH_ON_BOTH' the endpoint moved in X when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_AXIS][X], 0, tolerance,
                              "In test case 'BACKLASH_ON_AXIS' the endpoint moved in X when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_AXIS2][X], 0, tolerance,
                              "In test case 'BACKLASH_ON_AXIS2 the endpoint moved in X when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_NEITHER][X], 0, tolerance,
                              "In test case 'BACKLASH_ON_NEITHER' the endpoint moved in X when shouldn't.");
    ts_assert_double_in_delta(end_point_position[REFERENCE_HINGE2JOINT][X], 0, tolerance,
                              "In test case 'REFERENCE_HINGE2JOINT' the endpoint moved in X when shouldn't.");

    // keep track of reference position (minimum and maximum value of Z, adjusted by offset)
    if (end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0 > reference_max_z)
      reference_max_z = end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0;
    if (end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0 < reference_min_z)
      reference_min_z = end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0;

    // Test: no endpoint should swing beyond neither the current nor the maximal value reached by the reference.
    // check against the maxima
    tolerance = 1e-8;
    ts_assert_double_is_bigger(reference_max_z + tolerance, end_point_position[BACKLASH_ON_BOTH][Z],
                               "In test case 'BACKLASH_ON_BOTH' the endpoint breached the reference's maximal Z position.");
    ts_assert_double_is_bigger(reference_max_z + tolerance, end_point_position[BACKLASH_ON_AXIS][Z] + 0.25,
                               "In test case 'BACKLASH_ON_AXIS' the endpoint breached reference's maximal Z position.");
    ts_assert_double_is_bigger(reference_max_z + tolerance, end_point_position[BACKLASH_ON_AXIS2][Z] + 0.50,
                               "In test case 'BACKLASH_ON_AXIS2' the endpoint breached reference's maximal Z position.");
    ts_assert_double_is_bigger(reference_max_z + tolerance, end_point_position[BACKLASH_ON_NEITHER][Z] + 0.75,
                               "In test case 'BACKLASH_ON_NEITHER' the endpoint breached reference's maximal Z position.");

    // check against the minima
    tolerance = 1e-10;
    ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_BOTH][Z], reference_min_z - tolerance,
                               "In test case 'BACKLASH_ON_BOTH' the endpoint breached reference's minimal Z position.");
    ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_AXIS][Z] + 0.25, reference_min_z - tolerance,
                               "In test case 'BACKLASH_ON_AXIS' the endpoint breached reference's minimal Z position.");
    ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_AXIS2][Z] + 0.50, reference_min_z - tolerance,
                               "In test case 'BACKLASH_ON_AXIS2' the endpoint breached reference's minimal Z position.");
    ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_NEITHER][Z] + 0.75, reference_min_z - tolerance,
                               "In test case 'BACKLASH_ON_NEITHER' the endpoint breached reference's minimal Z position.");

    // TEST: assert if behavior is consistent (those with backlash behave all the same, those without as well)
    // (BACKLASH_ON_BOTH, BACKLASH_ON_AXIS2) should behave in the same manner
    tolerance = 1e-7;
    ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_BOTH][X], end_point_position[BACKLASH_ON_BOTH][Y],
                            end_point_position[BACKLASH_ON_BOTH][Z], end_point_position[BACKLASH_ON_AXIS2][X],
                            end_point_position[BACKLASH_ON_AXIS2][Y], end_point_position[BACKLASH_ON_AXIS2][Z] + 0.50,
                            tolerance, "'BACKLASH_ON_BOTH' and 'BACKLASH_ON_AXIS2' are not behaving in the same manner.");

    // (BACKLASH_ON_AXIS, BACKLASH_ON_NEITHER, REFERENCE_HINGE2JOINT) should behave in the same manner
    ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_AXIS][X], end_point_position[BACKLASH_ON_AXIS][Y],
                            end_point_position[BACKLASH_ON_AXIS][Z] + 0.25, end_point_position[REFERENCE_HINGE2JOINT][X],
                            end_point_position[REFERENCE_HINGE2JOINT][Y], end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0,
                            tolerance, "'BACKLASH_ON_AXIS' and 'REFERENCE_HINGE2JOINT' not behaving in the same manner.");

    ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_NEITHER][X], end_point_position[BACKLASH_ON_NEITHER][Y],
                            end_point_position[BACKLASH_ON_NEITHER][Z] + 0.75, end_point_position[REFERENCE_HINGE2JOINT][X],
                            end_point_position[REFERENCE_HINGE2JOINT][Y], end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0,
                            tolerance, "'BACKLASH_ON_NEITHER' and 'REFERENCE_HINGE2JOINT' not behaving in the same manner.");

    // the first group should behave differently from the second
    const double minimal_difference = 1e-3;                      // we don't want them to be similar at all
    if (wb_robot_get_time() > 0.5 && wb_robot_get_time() < 2.5)  // sufficient to test that they start differently
      ts_assert_double_is_bigger(
        fabs(end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0) - fabs(end_point_position[BACKLASH_ON_BOTH][Z]),
        minimal_difference, "'BACKLASH_ON_BOTH' and 'REFERENCE_HINGE2JOINT' behave similarly but shouldn't.");
  }

  wb_supervisor_simulation_reset();
  wb_supervisor_node_restart_controller(robot);

  wb_robot_step(TIME_STEP);

  double reference_min_x = INFINITY;
  double reference_max_x = -INFINITY;

  // Test: in this test only axis is actuated, the joints with no backlash in this direction should behave like the reference.
  outbuffer[0] = VELOCITY;
  outbuffer[1] = 0.0f;
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    // send command
    wb_emitter_send(emitter, outbuffer, 2 * sizeof(float));
    if (wb_robot_get_time() <= TIME_STEP / 1000)
      continue;  // give controllers time to receive commands and start
    // monitoring
    end_point_position[BACKLASH_ON_BOTH] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_BOTH]);
    end_point_position[BACKLASH_ON_AXIS] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_AXIS]);
    end_point_position[BACKLASH_ON_AXIS2] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_AXIS2]);
    end_point_position[BACKLASH_ON_NEITHER] = wb_supervisor_node_get_position(end_point_node[BACKLASH_ON_NEITHER]);
    end_point_position[REFERENCE_HINGE2JOINT] = wb_supervisor_node_get_position(end_point_node[REFERENCE_HINGE2JOINT]);

    // Test: verify that no endpoint changes in the Z direction
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_BOTH][Z], 0, 1e-10,
                              "In test case 'BACKLASH_ON_BOTH' the endpoint moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_AXIS][Z], -0.25, 1e-10,
                              "In test case 'BACKLASH_ON_AXIS' the endpoint moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_AXIS2][Z], -0.50, 1e-10,
                              "In test case 'BACKLASH_ON_AXIS2 the endpoint moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[BACKLASH_ON_NEITHER][Z], -0.75, 1e-10,
                              "In test case 'BACKLASH_ON_NEITHER' the endpoint moved in Z when shouldn't.");
    ts_assert_double_in_delta(end_point_position[REFERENCE_HINGE2JOINT][Z], -1.0, 1e-10,
                              "In test case 'REFERENCE_HINGE2JOINT' the endpoint moved in Z when shouldn't.");

    // keep track of reference position (minimum and maximum value of X)
    if (end_point_position[REFERENCE_HINGE2JOINT][X] > reference_max_x)
      reference_max_x = end_point_position[REFERENCE_HINGE2JOINT][X];
    if (end_point_position[REFERENCE_HINGE2JOINT][X] < reference_min_x)
      reference_min_x = end_point_position[REFERENCE_HINGE2JOINT][X];

    // TEST: no endpoint should swing beyond neither the current nor the maximal value reached by the reference
    // check against the maxima
    double tolerance = 1e-8;
    ts_assert_double_is_bigger(reference_max_x + tolerance, end_point_position[BACKLASH_ON_BOTH][X],
                               "In test case 'BACKLASH_ON_BOTH' the endpoint breached the reference's maximal X position.");
    ts_assert_double_is_bigger(reference_max_x + tolerance, end_point_position[BACKLASH_ON_AXIS][X],
                               "In test case 'BACKLASH_ON_AXIS' the endpoint breached reference's maximal X position.");
    ts_assert_double_is_bigger(reference_max_x + tolerance, end_point_position[BACKLASH_ON_AXIS2][X],
                               "In test case 'BACKLASH_ON_AXIS2' the endpoint breached reference's maximal X position.");
    ts_assert_double_is_bigger(reference_max_x + tolerance, end_point_position[BACKLASH_ON_NEITHER][X],
                               "In test case 'BACKLASH_ON_NEITHER' the endpoint breached reference's maximal X position.");

    // check against the minima
    tolerance = 1e-10;
    ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_BOTH][X], reference_min_x - tolerance,
                               "In test case 'BACKLASH_ON_BOTH' the endpoint breached reference's minimal X position.");
    ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_AXIS][X], reference_min_x - tolerance,
                               "In test case 'BACKLASH_ON_AXIS' the endpoint reference's minimal X position.");
    ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_AXIS2][X], reference_min_x - tolerance,
                               "In test case 'BACKLASH_ON_AXIS2' the endpoint reference's minimal X position.");
    ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_NEITHER][X], reference_min_x - tolerance,
                               "In test case 'BACKLASH_ON_NEITHER' the endpoint reference's minimal X position.");

    // TEST: assert if behavior is consistent (those with backlash behave all the same, those without as well)
    // (BACKLASH_ON_BOTH, BACKLASH_ON_AXIS) should behave in the same manner
    tolerance = 1e-8;
    ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_BOTH][X], end_point_position[BACKLASH_ON_BOTH][Y],
                            end_point_position[BACKLASH_ON_BOTH][Z], end_point_position[BACKLASH_ON_AXIS][X],
                            end_point_position[BACKLASH_ON_AXIS][Y], end_point_position[BACKLASH_ON_AXIS][Z] + 0.25, tolerance,
                            "'BACKLASH_ON_BOTH' and 'BACKLASH_ON_AXIS' are not behaving in the same manner.");

    // (BACKLASH_ON_AXIS2, BACKLASH_ON_NEITHER, REFERENCE_HINGE2JOINT) should behave in the same manner
    ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_AXIS2][X], end_point_position[BACKLASH_ON_AXIS2][Y],
                            end_point_position[BACKLASH_ON_AXIS2][Z] + 0.50, end_point_position[REFERENCE_HINGE2JOINT][X],
                            end_point_position[REFERENCE_HINGE2JOINT][Y], end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0,
                            tolerance, "'BACKLASH_ON_AXIS2' and 'REFERENCE_HINGE2JOINT' not behaving in the same manner.");

    ts_assert_vec3_in_delta(end_point_position[BACKLASH_ON_NEITHER][X], end_point_position[BACKLASH_ON_NEITHER][Y],
                            end_point_position[BACKLASH_ON_NEITHER][Z] + 0.75, end_point_position[REFERENCE_HINGE2JOINT][X],
                            end_point_position[REFERENCE_HINGE2JOINT][Y], end_point_position[REFERENCE_HINGE2JOINT][Z] + 1.0,
                            tolerance, "'BACKLASH_ON_NEITHER' and 'REFERENCE_HINGE2JOINT' not behaving in the same manner.");

    // the first group should behave differently from the second
    const double minimal_difference = 1e-3;                      // we don't want them to be similar at all
    if (wb_robot_get_time() > 0.5 && wb_robot_get_time() < 2.5)  // sufficient to test that they start differently
      ts_assert_double_is_bigger(
        fabs(end_point_position[REFERENCE_HINGE2JOINT][X]) - fabs(end_point_position[BACKLASH_ON_BOTH][X]), minimal_difference,
        "'BACKLASH_ON_BOTH' and 'REFERENCE_HINGE2JOINT' behave similarly but shouldn't.");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
