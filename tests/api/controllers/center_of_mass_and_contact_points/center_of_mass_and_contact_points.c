#include <math.h>
#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"
#define TIME_STEP 32

static const double THRESHOLD = 1e-5;
static const int REFERENCE_NUMBER_OF_CONTACT_POINTS = 16;
static const int ADDITIONAL_CONTACT_POINTS_NUMBER = 4;
static const double REFERENCE_COM[3] = {-0.000000, 0.067383, 0.000000};
static const double REFERENCE_CONTACT_POINTS[48] = {
  0.010000,  -0.000112, -0.025000, 0.025000,  -0.000112, -0.040000, 0.040000,  -0.000112, -0.025000, 0.025000,
  -0.000112, -0.010000, -0.040000, -0.000112, -0.025000, -0.025000, -0.000112, -0.040000, -0.010000, -0.000112,
  -0.025000, -0.025000, -0.000112, -0.010000, -0.040000, -0.000112, 0.025000,  -0.025000, -0.000112, 0.010000,
  -0.010000, -0.000112, 0.025000,  -0.025000, -0.000112, 0.040000,  0.010000,  -0.000112, 0.025000,  0.025000,
  -0.000112, 0.010000,  0.040000,  -0.000112, 0.025000,  0.025000,  -0.000112, 0.040000};

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  wb_robot_step(TIME_STEP);

  const WbNodeRef root = wb_supervisor_node_get_root();
  const WbFieldRef children = wb_supervisor_node_get_field(root, "children");
  ts_assert_boolean_equal(wb_supervisor_field_get_type(children) == WB_MF_NODE, "Children has not the WB_MF_NODE type");

  const int n = wb_supervisor_field_get_count(children);
  ts_assert_boolean_equal(n == 8, "Counting the number of root's children failed (found=%d, expected=8)", n);

  const WbNodeRef node = wb_supervisor_field_get_mf_node(children, 6);

  // center of mass checks
  const double *com = wb_supervisor_node_get_center_of_mass(node);
  double epsilon = 0.0;
  int i, j;
  for (i = 0; i < 3; i++)
    epsilon += fabs(com[i] - REFERENCE_COM[i]);

  ts_assert_boolean_equal(epsilon <= THRESHOLD, "The center of mass position does not match with the reference position.");

  // contact points (root only) checks
  int number_of_contact_points = wb_supervisor_node_get_number_of_contact_points(node, false);

  ts_assert_boolean_equal(number_of_contact_points == REFERENCE_NUMBER_OF_CONTACT_POINTS, "Wrong number of contact points.");

  for (i = 0; i < REFERENCE_NUMBER_OF_CONTACT_POINTS; i++) {
    for (j = 0; j < 3; j++)
      epsilon += fabs(wb_supervisor_node_get_contact_point(node, i)[j] - REFERENCE_CONTACT_POINTS[3 * i + j]);
  }
  epsilon = 0.0;
  ts_assert_boolean_equal(epsilon <= THRESHOLD, "The contact points positions do not match the reference positions.");

  // contact points (descendants included) checks
  number_of_contact_points = wb_supervisor_node_get_number_of_contact_points(node, true);
  ts_assert_boolean_equal(number_of_contact_points == REFERENCE_NUMBER_OF_CONTACT_POINTS + ADDITIONAL_CONTACT_POINTS_NUMBER,
                          "Wrong number of contact points when including descendants.");

  ts_assert_boolean_equal(wb_supervisor_node_get_contact_point_node(node, 0) == node,
                          "First contact point should belong to the main node.");

  ts_assert_boolean_equal(wb_supervisor_node_get_contact_point_node(
                            node, REFERENCE_NUMBER_OF_CONTACT_POINTS + ADDITIONAL_CONTACT_POINTS_NUMBER - 1) ==
                            wb_supervisor_node_get_from_def("SECONDARY_SOLID"),
                          "Last contact point should belong to the 'SECONDARY_SOLID' node.");

  // static balance checks
  const bool stable = wb_supervisor_node_get_static_balance(node);

  ts_assert_boolean_equal(
    stable, "The support polygon stability test returns an unstable state whereas the reference state is stable.");

  ts_send_success();
  return EXIT_SUCCESS;
}
