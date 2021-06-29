#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);  // give the controller args

  WbNodeRef probe_proto_node = wb_supervisor_node_get_from_def("PROBE_PROTO");
  WbNodeRef probe_robot_node = wb_supervisor_node_get_from_def("PROBE_ROBOT");

  WbFieldRef probe_proto_translation_field = wb_supervisor_node_get_field(probe_proto_node, "translation");
  WbFieldRef probe_robot_translation_field = wb_supervisor_node_get_field(probe_robot_node, "translation");

  wb_robot_step(TIME_STEP);

  int proto_number_contact_points, robot_number_contact_points;
  wb_supervisor_node_get_contact_points(probe_proto_node, true, &proto_number_contact_points);
  wb_supervisor_node_get_contact_points(probe_robot_node, true, &robot_number_contact_points);

  wb_robot_step(TIME_STEP);

  ts_assert_int_equal(proto_number_contact_points, 0,
                      "At the initial position of the proto probe the number of contact points should be 0 but isn't.");
  ts_assert_int_equal(robot_number_contact_points, 0,
                      "At the initial position of the robot probe the number of contact points should be 0 but isn't.");

  // set probe position in the first zone of conflict, collision should occur
  const double probe_proto_first_contact_position[3] = {0.5, 0, 0};
  const double probe_robot_first_contact_position[3] = {0.5, 0, 2};

  wb_supervisor_field_set_sf_vec3f(probe_proto_translation_field, probe_proto_first_contact_position);
  wb_supervisor_field_set_sf_vec3f(probe_robot_translation_field, probe_robot_first_contact_position);

  wb_robot_step(TIME_STEP);

  wb_supervisor_node_get_contact_points(probe_proto_node, true, &proto_number_contact_points);
  wb_supervisor_node_get_contact_points(probe_robot_node, true, &robot_number_contact_points);

  ts_assert_int_equal(proto_number_contact_points, 4,
                      "The first contact position for the proto probe should have 4 contact points but doesn't.");
  ts_assert_int_equal(robot_number_contact_points, 4,
                      "The first contact position for the robot probe should have 4 contact points but doesn't.");

  wb_robot_step(10 * TIME_STEP);

  // force a regeneration of the boundingObject, making it smaller
  WbNodeRef regenerable_proto_box_node = wb_supervisor_node_get_from_def("REGENERABLE_BOX_IN_PROTO");
  WbNodeRef regenerable_robot_box_node = wb_supervisor_node_get_from_def("REGENERABLE_BOX_IN_ROBOT");

  WbFieldRef proto_size_field = wb_supervisor_node_get_field(regenerable_proto_box_node, "size");
  WbFieldRef robot_size_field = wb_supervisor_node_get_field(regenerable_robot_box_node, "size");

  wb_supervisor_field_set_sf_float(proto_size_field, 0.5);
  wb_supervisor_field_set_sf_float(robot_size_field, 0.5);

  // test previous contact position, should no longer collide
  wb_supervisor_field_set_sf_vec3f(probe_proto_translation_field, probe_proto_first_contact_position);
  wb_supervisor_field_set_sf_vec3f(probe_robot_translation_field, probe_robot_first_contact_position);

  wb_robot_step(10 * TIME_STEP);

  wb_supervisor_node_get_contact_points(probe_proto_node, true, &proto_number_contact_points);
  wb_supervisor_node_get_contact_points(probe_robot_node, true, &robot_number_contact_points);

  ts_assert_int_equal(
    proto_number_contact_points, 0,
    "After regeneration, for the proto probe the first contact position should have no contact points, but does.");
  ts_assert_int_equal(
    robot_number_contact_points, 0,
    "After regeneration, for the robot probe the first contact position should have no contact points, but does.");

  // set probe position in the second zone of conflict, collision should occur
  const double proto_second_contact_position[3] = {0.25, 0, 0};
  const double robot_second_contact_position[3] = {0.25, 0, 2};

  wb_supervisor_field_set_sf_vec3f(probe_proto_translation_field, proto_second_contact_position);
  wb_supervisor_field_set_sf_vec3f(probe_robot_translation_field, robot_second_contact_position);

  wb_robot_step(TIME_STEP);

  wb_supervisor_node_get_contact_points(probe_proto_node, true, &proto_number_contact_points);
  wb_supervisor_node_get_contact_points(probe_robot_node, true, &robot_number_contact_points);

  ts_assert_int_equal(proto_number_contact_points, 4,
                      "The second contact position for the proto probe should have 4 contact points but doesn't.");
  ts_assert_int_equal(robot_number_contact_points, 4,
                      "The second contact position for the robot probe should have 4 contact points but doesn't.");

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
