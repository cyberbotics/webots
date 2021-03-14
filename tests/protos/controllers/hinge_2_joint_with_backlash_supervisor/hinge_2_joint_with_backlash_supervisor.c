#include <stdio.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16
#define TEST_DURATION 3.0f
#define VELOCITY 0.2f
#define ANGLE_LIMIT 0.523599f

// enum test_cases { SIMPLE_HINGE2JOINT, BACKLASH_02_AND_02, BACKLASH_02_AND_00, BACKLASH_00_AND_02, BACKLASH_00_AND_00 };
typedef enum { BACKLASH_ON_BOTH, BACKLASH_ON_AXIS, BACKLASH_ON_AXIS2, BACKLASH_ON_NEITHER, REFERENCE_HINGE2JOINT } test_cases;
enum coordinate { X, Y, Z };

const char *test_name(test_cases test) {
  switch (test) {
    case BACKLASH_ON_BOTH:
      return "BACKLASH_ON_BOTH";
    case BACKLASH_ON_AXIS:
      return "BACKLASH_ON_AXIS";
    case BACKLASH_ON_AXIS2:
      return "BACKLASH_ON_AXIS2";
    case BACKLASH_ON_NEITHER:
      return "BACKLASH_ON_NEITHER";
    case REFERENCE_HINGE2JOINT:
      return "REFERENCE_HINGE2JOINT";
    default:
      return "UNDEFINED";
  }
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef end_point_node[5];
  end_point_node[BACKLASH_ON_BOTH] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_BOTH");
  end_point_node[BACKLASH_ON_AXIS] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_AXIS");
  end_point_node[BACKLASH_ON_AXIS2] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_AXIS2");
  end_point_node[BACKLASH_ON_NEITHER] = wb_supervisor_node_get_from_def("END_POINT_CASE_BACKLASH_ON_NEITHER");
  end_point_node[REFERENCE_HINGE2JOINT] = wb_supervisor_node_get_from_def("END_POINT_CASE_REFERENCE_HINGE2JOINT");

  // classic Hinge2Joint node used as reference
  WbDeviceTag reference_hinge2joint_sensor = wb_robot_get_device("reference_hinge2joint_sensor");
  WbDeviceTag reference_hinge2joint_sensor2 = wb_robot_get_device("reference_hinge2joint_sensor2");
  wb_position_sensor_enable(reference_hinge2joint_sensor, TIME_STEP);
  wb_position_sensor_enable(reference_hinge2joint_sensor2, TIME_STEP);
  // all motors receive the same command
  WbDeviceTag motor[5];
  WbDeviceTag motor2[5];
  char names_motor[5][30] = {"backlash_on_both_motor", "backlash_on_axis_motor", "backlash_on_axis2_motor",
                             "backlash_on_neither_motor", "reference_hinge2joint_motor"};
  char names_motor2[5][30] = {"backlash_on_both_motor2", "backlash_on_axis_motor2", "backlash_on_axis2_motor2",
                              "backlash_on_neither_motor2", "reference_hinge2joint_motor2"};

  for (int i = 0; i < 5; ++i) {
    motor[i] = wb_robot_get_device(names_motor[i]);
    motor2[i] = wb_robot_get_device(names_motor2[i]);
    wb_motor_set_position(motor[i], INFINITY);
    wb_motor_set_position(motor2[i], INFINITY);
  }

  double reference_min_y = INFINITY;
  double reference_max_y = -INFINITY;
  const double *end_point_position[5];

  // Test: In this test only axis2 is actuated, the joints with no backlash in this direction should behave
  // like the reference joint
  for (int i = 0; i < 5; ++i) {
    wb_motor_set_velocity(motor[i], 0);
    wb_motor_set_velocity(motor2[i], VELOCITY);
  }

  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    // command
    double reference_hinge2joint_position2 = wb_position_sensor_get_value(reference_hinge2joint_sensor2);

    if (reference_hinge2joint_position2 > ANGLE_LIMIT) {
      for (int i = 0; i < 5; ++i) {
        wb_motor_set_velocity(motor2[i], -VELOCITY);
      }
    }
    if (reference_hinge2joint_position2 < -ANGLE_LIMIT) {
      for (int i = 0; i < 5; ++i) {
        wb_motor_set_velocity(motor2[i], VELOCITY);
      }
    }

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
    const double eps = 1e-8;  // due to stop penetration some leeway is necessary
    // check against highest point
    ts_assert_double_is_bigger(reference_max_y + eps, end_point_position[BACKLASH_ON_BOTH][Y],
                               "In test case 'BACKLASH_ON_BOTH' the endpoint moved above the reference.");
    ts_assert_double_is_bigger(reference_max_y + eps, end_point_position[BACKLASH_ON_AXIS][Y],
                               "In test case 'BACKLASH_ON_AXIS' the endpoint moved above the reference.");
    ts_assert_double_is_bigger(reference_max_y + eps, end_point_position[BACKLASH_ON_AXIS2][Y],
                               "In test case 'BACKLASH_ON_AXIS2' the endpoint moved above the reference.");
    ts_assert_double_is_bigger(reference_max_y + eps, end_point_position[BACKLASH_ON_NEITHER][Y],
                               "In test case 'BACKLASH_ON_NEITHER' the endpoint moved above the reference.");
    // check against lowest point
    if (wb_robot_get_time() > 0.6) {  // only makes sense to check this after the ones with backlash begin to move
      ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_BOTH][Y], reference_min_y - eps,
                                 "In test case 'BACKLASH_ON_BOTH' the endpoint moved below the reference.");
      ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_AXIS][Y], reference_min_y - eps,
                                 "In test case 'BACKLASH_ON_AXIS' the endpoint moved below the reference.");
      ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_AXIS2][Y], reference_min_y - eps,
                                 "In test case 'BACKLASH_ON_AXIS2' the endpoint moved below the reference.");
      ts_assert_double_is_bigger(end_point_position[BACKLASH_ON_NEITHER][Y], reference_min_y - eps,
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
    if (wb_robot_get_time() < 2.5) {  // sufficient to test that they start differently
      ts_assert_double_is_bigger(fabs(end_point_position[BACKLASH_ON_BOTH][Y] - end_point_position[REFERENCE_HINGE2JOINT][Y]),
                                 1e-5, "'BACKLASH_ON_BOTH' and 'REFERENCE_HINGE2JOINT' behave similarly but shouldn't.");
    }
  }

  wb_supervisor_simulation_reset_physics();
    wb_robot_step(10*TIME_STEP);

  wb_supervisor_simulation_reset();
  // wb_supervisor_simulation_reset_physics();
  wb_robot_step(100*TIME_STEP);


  // Test: In this test only axis is actuated, the joints with no backlash in this direction
  // should behave like the reference joint
  for (int i = 0; i < 5; ++i) {
    wb_motor_set_position(motor[i], INFINITY);
    wb_motor_set_position(motor2[i], INFINITY);
    wb_motor_set_velocity(motor[i], 0);
    wb_motor_set_velocity(motor2[i], VELOCITY);
  }
  
    wb_robot_step(10000*TIME_STEP);


  ts_send_success();
  return EXIT_SUCCESS;
}
