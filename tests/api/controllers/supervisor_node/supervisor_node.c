/*
 * Description:  Test Supervisor device API
 *               This file contains tests of the methods relative to nodes,
 *               i.e. functions starting with wb_supervisor_node_*:
 *                 wb_supervisor_node_get_root
 *                 wb_supervisor_node_get_self
 *                 wb_supervisor_node_get_from_def
 *                 wb_supervisor_node_get_type
 *                 wb_supervisor_node_get_type_name
 *                 wb_supervisor_node_get_field
 *                 wb_supervisor_node_get_position
 *                 wb_supervisor_node_get_orientation
 *                 wb_supervisor_node_export_string
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <math.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef root, self, node, proto, solid, material, robot, reset_target;
  WbFieldRef children, field, controller;
  const char *charArray;
  const double *doubleArray;
  int i;
  double state_position[3];
  double state_rotation[4];

  WbDeviceTag sick_lidar = wb_robot_get_device("Sick LMS 291");
  WbDeviceTag compass = wb_robot_get_device("compass");

  double time = wb_robot_get_time();
  ts_assert_boolean_equal(time == 0.0, "Starting time is wrong. Expected=0.0. Received=%f\n", time);

  charArray = wb_robot_get_name();
  if (strcmp(charArray, "supervisor") != 0) {
    // check supervisors functions on a non-supervisor node

    // root
    root = wb_supervisor_node_get_root();
    ts_assert_pointer_null(root, "A non supervisor node should return NULL when requesting the root node");

    // get from def
    node = wb_supervisor_node_get_from_def("BOX");
    ts_assert_pointer_null(node, "A non supervisor node should return NULL when requesting the \"BOX\" node");

    // get from def
    node = wb_supervisor_node_get_from_id(1);
    ts_assert_pointer_null(node, "A non supervisor node should return NULL when requesting the node having the '1' unique id");

    ts_send_success();
    return EXIT_SUCCESS;
  }

  // root
  root = wb_supervisor_node_get_root();
  ts_assert_pointer_not_null(root, "Root node is not found");

  i = wb_supervisor_node_get_type(root);
  ts_assert_int_equal(i, WB_NODE_GROUP, "Root node should have type %d not %d", WB_NODE_GROUP, i);

  charArray = wb_supervisor_node_get_type_name(root);
  ts_assert_string_equal(charArray, "Group", "Root node should have type name \"%s\" not \"%s\"", "Group", charArray);

  children = wb_supervisor_node_get_field(root, "children");
  ts_assert_boolean_equal(wb_supervisor_field_get_type(children) == WB_MF_NODE, "Children has not the WB_MF_NODE type");

  i = wb_supervisor_field_get_count(children);
  ts_assert_int_equal(i, 12, "Root node should have %d children not %d", 12, i);

  node = wb_supervisor_node_get_from_id(-1);  // invalid
  ts_assert_pointer_null(node, "Invalid test of wb_supervisor_node_get_from_id() failed");

  // Note: the 3 following tests will be true as the unique id of the nodes
  // is incremented by 1 for each node, starting from 0 for the root node
  node = wb_supervisor_node_get_from_id(0);  // root node
  i = wb_supervisor_node_get_type(node);
  ts_assert_int_equal(i, WB_NODE_GROUP, "Root node should have type %d not %d", WB_NODE_GROUP, i);
  i = wb_supervisor_node_get_id(node);
  ts_assert_int_equal(i, 0, "wb_supervisor_node_get_id(Node(0)): id doesn't match.");

  node = wb_supervisor_node_get_from_id(1);
  i = wb_supervisor_node_get_type(node);
  ts_assert_int_equal(i, WB_NODE_WORLD_INFO, "Root node should have type %d not %d", WB_NODE_WORLD_INFO, i);
  i = wb_supervisor_node_get_id(node);
  ts_assert_int_equal(i, 1, "wb_supervisor_node_get_id(Node(1)): id doesn't match.");

  node = wb_supervisor_node_get_from_id(2);
  i = wb_supervisor_node_get_type(node);
  ts_assert_int_equal(i, WB_NODE_VIEWPOINT, "Root node should have type %d not %d", WB_NODE_VIEWPOINT, i);
  i = wb_supervisor_node_get_id(node);
  ts_assert_int_equal(i, 2, "wb_supervisor_node_get_id(Node(2)): id doesn't match.");

  // solid node
  robot = wb_supervisor_node_get_from_def("ROBOT");
  ts_assert_pointer_not_null(robot, "\"ROBOT\" node is not found");

  i = wb_supervisor_node_get_type(robot);
  ts_assert_int_equal(i, WB_NODE_ROBOT, "\"ROBOT\" node should have type %d not %d", WB_NODE_ROBOT, i);

  charArray = wb_supervisor_node_get_type_name(robot);
  ts_assert_string_equal(charArray, "Robot", "\"ROBOT\" node should have type \"%s\" not \"%s\"", "Robot", charArray);

  node = wb_supervisor_node_get_parent_node(robot);
  ts_assert_boolean_equal(wb_supervisor_node_get_id(node) == wb_supervisor_node_get_id(root),
                          "Parent node of robot should be the root node (found id = %d).", wb_supervisor_node_get_id(node));

  // solid node
  solid = wb_supervisor_node_get_from_def("BOX");
  ts_assert_pointer_not_null(solid, "\"BOX\" node is not found");

  i = wb_supervisor_node_get_type(solid);
  ts_assert_int_equal(i, WB_NODE_SOLID, "\"BOX\" node should have type %d not %d", WB_NODE_SOLID, i);

  charArray = wb_supervisor_node_get_type_name(solid);
  ts_assert_string_equal(charArray, "Solid", "\"BOX\" node should have type \"%s\" not \"%s\"", "Solid", charArray);

  node = wb_supervisor_node_get_parent_node(solid);
  ts_assert_boolean_equal(wb_supervisor_node_get_id(node) == wb_supervisor_node_get_id(robot),
                          "Parent node of solid should be the robot node (found id = %d).", wb_supervisor_node_get_id(node));

  // material node
  material = wb_supervisor_node_get_from_def("CAPSULE_MATERIAL");
  ts_assert_pointer_not_null(material, "\"CAPSULE_MATERIAL\" node is not found");

  i = wb_supervisor_node_get_type(material);
  ts_assert_int_equal(i, WB_NODE_MATERIAL, "\"CAPSULE_MATERIAL\" node should have type %d not %d", WB_NODE_MATERIAL, i);

  charArray = wb_supervisor_node_get_type_name(material);
  ts_assert_string_equal(charArray, "Material", "\"CAPSULE_MATERIAL\" node should have type \"%s\" not \"%s\"", "Material",
                         charArray);

  // solid proto node
  proto = wb_supervisor_node_get_from_def("CAN");
  ts_assert_pointer_not_null(proto, "\"CAN\" proto node is not found");

  i = wb_supervisor_node_get_type(proto);
  ts_assert_int_equal(i, WB_NODE_SOLID, "\"CAN\" proto node should have type %d not %d", WB_NODE_SOLID, i);

  charArray = wb_supervisor_node_get_type_name(proto);
  ts_assert_string_equal(charArray, "Can", "\"CAN\" proto node has type \"%s\" not \"%s\"", "Can", charArray);

  charArray = wb_supervisor_node_get_base_type_name(proto);
  ts_assert_string_equal(charArray, "Solid", "\"CAN\" proto node has base type \"%s\" not \"%s\"", "Solid", charArray);

  // device node
  node = wb_supervisor_node_get_from_def("DISTANCE_SENSOR");
  ts_assert_pointer_not_null(node, "\"DISTANCE_SENSOR\" node is not found");

  i = wb_supervisor_node_get_type(node);
  ts_assert_int_equal(i, WB_NODE_DISTANCE_SENSOR, "\"DISTANCE_SENSOR\" node should have type %d not %d",
                      WB_NODE_DISTANCE_SENSOR, i);

  charArray = wb_supervisor_node_get_type_name(node);
  ts_assert_string_equal(charArray, "DistanceSensor", "\"DISTANCE_SENSOR\" node should have type \"%s\" not \"%s\"",
                         "DistanceSensor", charArray);

  node = wb_supervisor_node_get_from_def("CAPSULE_GEOMETRY");
  ts_assert_pointer_not_null(node, "\"CAPSULE_GEOMETRY\" node is not found");

  i = wb_supervisor_node_get_type(node);
  ts_assert_int_equal(i, WB_NODE_CAPSULE, "\"CAPSULE_GEOMETRY\" node should have type %d not %d", WB_NODE_CAPSULE, i);

  charArray = wb_supervisor_node_get_type_name(node);
  ts_assert_string_equal(charArray, "Capsule", "\"CAPSULE_GEOMETRY\" node should have type \"%s\" not \"%s\"", "Capsule",
                         charArray);

  // cone node
  node = wb_supervisor_node_get_from_def("CONE_GEOMETRY");
  ts_assert_pointer_not_null(node, "\"CAPSULE_GEOMETRY\" node is not found");

  i = wb_supervisor_node_get_type(node);
  ts_assert_int_equal(i, WB_NODE_CONE, "\"CONE_GEOMETRY\" node should have type %d not %d", WB_NODE_CONE, i);

  charArray = wb_supervisor_node_get_type_name(node);
  ts_assert_string_equal(charArray, "Cone", "\"CONE_GEOMETRY\" node should have type \"%s\" not \"%s\"", "Cone", charArray);

  // physics node
  node = wb_supervisor_node_get_from_def("SPHERE_PHYSICS");
  ts_assert_pointer_not_null(node, "\"SPHERE_PHYSICS\" node is not found");

  i = wb_supervisor_node_get_type(node);
  ts_assert_int_equal(i, WB_NODE_PHYSICS, "\"SPHERE_PHYSICS\" node should have type %d not %d", WB_NODE_PHYSICS, i);

  charArray = wb_supervisor_node_get_type_name(node);
  ts_assert_string_equal(charArray, "Physics", "\"SPHERE_PHYSICS\" node should have type \"%s\" not \"%s\"", "Physics",
                         charArray);

  // get field with invalid node
  field = wb_supervisor_node_get_field(NULL, "children");
  ts_assert_pointer_null(field, "If a field is requested for a NULL node, the result should be NULL");

  // get node with invalid name
  node = wb_supervisor_node_get_from_def("notExistingNode");
  ts_assert_pointer_null(node, "Requesting a node with an invalid name sould return NULL");

  node = wb_supervisor_node_get_from_def(NULL);
  ts_assert_pointer_null(node, "Requesting a node with a NULL name sould return NULL");

  // request invalid field
  field = wb_supervisor_node_get_field(proto, "notExistingFieldName");
  ts_assert_pointer_null(field, "If a field with an invalid name is requested for a proto node, the result should be NULL");
  field = wb_supervisor_node_get_field(solid, "");
  ts_assert_pointer_null(field, "If a field with an empty name is requested for a solid node, the result should be NULL");
  field = wb_supervisor_node_get_field(material, NULL);
  ts_assert_pointer_null(field, "If a field with a NULL name is requested for a solid node, the result should be NULL");

  // get position
  doubleArray = wb_supervisor_node_get_position(solid);
  ts_assert_vec3_in_delta(doubleArray[0], doubleArray[1], doubleArray[2], -0.2, 0.2, -0.2, 0.0001,
                          "Position of solid node should be [%f, %f, %f] not [%f, %f, %f]", -0.2, 0.2, -0.2, doubleArray[0],
                          doubleArray[1], doubleArray[2]);

  doubleArray = wb_supervisor_node_get_position(material);
  ts_assert_boolean_equal(isnan(doubleArray[0]) && isnan(doubleArray[1]) && isnan(doubleArray[2]),
                          "Position of material node should be NAN");

  doubleArray = wb_supervisor_node_get_position(proto);
  ts_assert_vec3_in_delta(doubleArray[0], doubleArray[1], doubleArray[2], -0.21, 0.031, 0.24, 0.001,
                          "Position of proto node should be [%f, %f, %f] not [%f, %f, %f]", -0.21, 0.031, 0.24, doubleArray[0],
                          doubleArray[1], doubleArray[2]);

  // get orientation
  doubleArray = wb_supervisor_node_get_orientation(solid);
  double expected0[9] = {0.921, 0.326, 0.214, -0.214, 0.881, -0.422, -0.326, 0.342, 0.881};
  ts_assert_doubles_in_delta(
    9, doubleArray, expected0, 0.001,
    "Orientation of solid node should be [%f, %f, %f; %f, %f, %f; %f, %f, %f] not [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
    expected0[0], expected0[1], expected0[2], expected0[3], expected0[4], expected0[5], expected0[6], expected0[7],
    expected0[8], doubleArray[0], doubleArray[1], doubleArray[2], doubleArray[3], doubleArray[4], doubleArray[5],
    doubleArray[6], doubleArray[7], doubleArray[8]);

  doubleArray = wb_supervisor_node_get_orientation(material);
  ts_assert_boolean_equal(isnan(doubleArray[0]) && isnan(doubleArray[1]) && isnan(doubleArray[2]) && isnan(doubleArray[3]) &&
                            isnan(doubleArray[4]) && isnan(doubleArray[5]) && isnan(doubleArray[6]) && isnan(doubleArray[7]) &&
                            isnan(doubleArray[8]),
                          "Orientation of material node should be NAN");

  doubleArray = wb_supervisor_node_get_orientation(proto);
  double expected1[9] = {0.957, 0.204, 0.207, 0.212, 0.000, -0.977, -0.199, 0.979, -0.043};
  ts_assert_doubles_in_delta(
    9, doubleArray, expected1, 0.001,
    "Orientation of proto node should be [%f, %f, %f; %f, %f, %f; %f, %f, %f] not [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
    expected1[0], expected1[1], expected1[2], expected1[3], expected1[4], expected1[5], expected1[6], expected1[7],
    expected1[8], doubleArray[0], doubleArray[1], doubleArray[2], doubleArray[3], doubleArray[4], doubleArray[5],
    doubleArray[6], doubleArray[7], doubleArray[8]);

  // self node
  self = wb_supervisor_node_get_self();
  ts_assert_pointer_not_null(self, "Self node is not found");

  i = wb_supervisor_node_get_type(self);
  ts_assert_int_equal(i, WB_NODE_ROBOT, "Self node should have type %d not %d", WB_NODE_ROBOT, i);

  ts_assert_boolean_equal(wb_robot_get_supervisor(), "'wb_robot_get_supervisor' should return true.");

  controller = wb_supervisor_node_get_field(self, "controller");
  ts_assert_pointer_not_null(controller, "Self node should have a controller field.");

  charArray = wb_supervisor_field_get_sf_string(controller);
  ts_assert_string_equal(charArray, "supervisor_node", "Self node should have controller field set to \"%s\" not \"%s\"",
                         "supervisor_node", charArray);

  wb_robot_step(TIME_STEP);

  // make sure this has been done in one step
  time = wb_robot_get_time();
  ts_assert_boolean_equal(time == 0.001 * TIME_STEP, "Ending time is wrong. Expected=%f. Received=%f\n", 0.001 * TIME_STEP,
                          time);

  wb_robot_step(TIME_STEP);

  reset_target = wb_supervisor_node_get_from_def("RESET_TARGET");
  wb_supervisor_node_save_state(reset_target, "custom_state");
  memcpy(state_rotation, wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(reset_target, "rotation")),
         4 * sizeof(double));
  memcpy(state_position, wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(reset_target, "translation")),
         3 * sizeof(double));

  WbNodeRef self_by_tag = wb_supervisor_node_get_from_device(0);
  ts_assert_pointer_not_null(self_by_tag, "Null node reference to self node by device tag");
  ts_assert_boolean_equal(self_by_tag == wb_supervisor_node_get_self(), "Invalid node reference to self node by tag");

  WbNodeRef compass_node = wb_supervisor_node_get_from_device(compass);
  ts_assert_pointer_not_null(compass_node, "Invalid compass node reference from device tag");
  ts_assert_string_equal(wb_supervisor_node_get_def(compass_node), "COMPASS", "Wrong compass node reference");

  WbNodeRef sick_node = wb_supervisor_node_get_from_device(sick_lidar);
  ts_assert_pointer_not_null(sick_node, "Invalid Sick node reference from device tag");
  WbFieldRef sick_translation_field = wb_supervisor_node_get_field(sick_node, "translation");
  ts_assert_pointer_not_null(sick_translation_field, "Invalid translation field for Sick node");
  const double *sick_translation = wb_supervisor_field_get_sf_vec3f(sick_translation_field);
  ts_assert_vec3_equal(sick_translation[0], sick_translation[1], sick_translation[2], 0.39, 0.1, 0.4,
                       "Invalid translation value for Sick node");

  wb_supervisor_node_load_state(reset_target, "custom_state");
  doubleArray = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(reset_target, "translation"));
  ts_assert_doubles_equal(3, state_position, doubleArray,
                          "Current solid's position is not equal to the solid's position on when the "
                          "wb_supervisor_node_save_state function was called");

  doubleArray = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(reset_target, "rotation"));
  ts_assert_doubles_equal(4, state_rotation, doubleArray,
                          "Current solid's rotation is not equal to the solid's rotation on when the "
                          "wb_supervisor_node_save_state function was called");

  node = wb_supervisor_node_get_from_def("CONE");
  ts_assert_string_contains(wb_supervisor_node_export_string(node), "DEF CONE Solid {", "The exported string is not valid");

  ts_send_success();
  return EXIT_SUCCESS;
}
