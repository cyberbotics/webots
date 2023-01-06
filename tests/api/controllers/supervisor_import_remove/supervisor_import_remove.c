#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag ds0 = wb_robot_get_device("ds0");
  WbDeviceTag ds1 = wb_robot_get_device("ds1");

  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

  wb_robot_step(4 * TIME_STEP);  // let time for the other controller to run

  wb_supervisor_node_remove(wb_supervisor_node_get_from_def("WORLDINFO"));

  double value0 = wb_distance_sensor_get_value(ds0);
  double value1 = wb_distance_sensor_get_value(ds1);
  ts_assert_double_is_bigger(750, value0, "Problem with initial state => ds0 can't see the obstacle");
  ts_assert_double_is_bigger(value1, 750, "Problem with initial state => ds1 can see an obstacle");

  WbFieldRef root_children_field = wb_supervisor_node_get_field(wb_supervisor_node_get_root(), "children");
  int initial_root_children_count = wb_supervisor_field_get_count(root_children_field);
  wb_supervisor_field_import_mf_node_from_string(root_children_field, 5,
                                                 "DEF SPHERE1 Solid { translation -0.15 0 0 children [ DEF SPHERE Shape { "
                                                 "geometry Sphere { radius 0.1 } } ] boundingObject USE SPHERE }");

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children_field), initial_root_children_count + 1,
                      "The number of children of the root node is not correct after the insertion of SPHERE1.");

  wb_robot_step(TIME_STEP);

  int root_children_count = wb_supervisor_field_get_count(root_children_field);
  wb_supervisor_field_import_mf_node_from_string(root_children_field, 5, "INVALID");

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children_field), root_children_count,
                      "The number of children of the root node should not change when importing an invalid node.");

  wb_robot_step(TIME_STEP);

  value1 = wb_distance_sensor_get_value(ds1);
  ts_assert_double_is_bigger(750, value1, "The newly imported sphere is not detected");

  WbNodeRef sphere0_node = wb_supervisor_node_get_from_def("SPHERE0");
  ts_assert_pointer_not_null(sphere0_node, "Invalid reference to node 'SPHERE0'");
  wb_supervisor_field_remove_mf(root_children_field, 4);

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children_field), initial_root_children_count,
                      "The number of children of the root node is not correct after the deletion of SPHERE0.");

  wb_robot_step(TIME_STEP);

  value0 = wb_distance_sensor_get_value(ds0);
  ts_assert_double_is_bigger(value0, 750, "The removed sphere is still detected");

  sphere0_node = wb_supervisor_node_get_from_def("SPHERE0");
  ts_assert_pointer_null(sphere0_node, "Reference to node 'SPHERE0' should now be invalid");

  wb_supervisor_field_import_mf_node_from_string(root_children_field, 5,
                                                 "DEF SPHERE2 Solid { translation 0.15 0 0 children [ DEF SPHERE Shape { "
                                                 "geometry Sphere { radius 0.1 } } ] boundingObject USE SPHERE }");

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children_field), initial_root_children_count + 1,
                      "The number of children of the root node is not correct after the insertion of SPHERE2.");

  wb_robot_step(2 * TIME_STEP);

  // Try to remove root
  // this is an invalid action: nothing should be applied and Webots should not crash
  wb_supervisor_node_remove(wb_supervisor_node_get_root());

  value0 = wb_distance_sensor_get_value(ds0);
  ts_assert_double_is_bigger(750, value0, "SPHERE2 is not detected");

  WbNodeRef sphere2_node = wb_supervisor_node_get_from_def("SPHERE2");
  ts_assert_pointer_not_null(sphere2_node, "Invalid reference to node 'SPHERE2'");
  wb_supervisor_node_remove(sphere2_node);

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children_field), initial_root_children_count,
                      "The number of children of the root node is not correct after the deletion of SPHERE2.");

  wb_robot_step(TIME_STEP);

  // Remove an already deleted node: invalid action
  wb_supervisor_node_remove(sphere2_node);
  ts_assert_int_equal(wb_supervisor_field_get_count(root_children_field), initial_root_children_count,
                      "The number of children of the root node is not correct after the deletion of SPHERE2.");

  wb_robot_step(TIME_STEP);

  value0 = wb_distance_sensor_get_value(ds0);
  ts_assert_double_is_bigger(value0, 750, "SPHERE2 is still detected");

  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1,
                                                 "DEF SPHERE3 Solid { translation 0.15 0 0 children [ DEF SPHERE Shape { "
                                                 "geometry Sphere { radius 0.1 } } ] boundingObject USE SPHERE }");

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children_field), initial_root_children_count + 1,
                      "The number of children of the root node is not correct after the insertion of SPHERE3.");

  wb_robot_step(TIME_STEP);

  value0 = wb_distance_sensor_get_value(ds0);
  ts_assert_double_is_bigger(750, value0, "SPHERE3 is not detected");

  WbNodeRef sphere3_node = wb_supervisor_field_get_mf_node(root_children_field, -1);
  ts_assert_pointer_not_null(sphere3_node, "Invalid reference to node 'SPHERE3'");
  wb_supervisor_field_remove_mf(root_children_field, -1);

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children_field), initial_root_children_count,
                      "The number of children of the root node is not correct after the deletion of SPHERE3.");

  wb_robot_step(TIME_STEP);

  value0 = wb_distance_sensor_get_value(ds0);
  ts_assert_double_is_bigger(value0, 750, "SPHERE3 is still detected");

  // test remove node from empty MFNode field
  WbNodeRef import_test_node = wb_supervisor_node_get_from_def("IMPORT_TEST");
  WbFieldRef import_children_field = wb_supervisor_node_get_field(import_test_node, "children");
  wb_supervisor_field_remove_mf(import_children_field, 0);

  // test import DEF node in a used ancestor DEF node -> check Webots doesn't crash
  wb_supervisor_field_import_mf_node_from_string(import_children_field, -1,
                                                 "Shape { geometry DEF MY_GEOM Box { size 0.1 0.1 0.1 } }");

  wb_robot_step(TIME_STEP);

  root_children_count = wb_supervisor_field_get_count(root_children_field);
  wb_supervisor_field_import_mf_node_from_string(root_children_field, 5, "Solid { } Solid { }");

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children_field), root_children_count + 2,
                      "The number of children of the root node should increase by two when importing two nodes.");

  // test internal update of libController node_list on node delete
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, "DEF SPHERE4 Solid {}");
  WbNodeRef sphere4 = wb_supervisor_field_get_mf_node(root_children_field, -1);
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, "DEF SPHERE5 Solid {}");
  WbNodeRef sphere5 = wb_supervisor_field_get_mf_node(root_children_field, -1);
  wb_robot_step(TIME_STEP);
  wb_supervisor_node_remove(sphere4);
  wb_robot_step(TIME_STEP);
  ts_assert_boolean_equal(wb_supervisor_field_get_mf_node(root_children_field, -1) == sphere5,
                          "WbNodeRef instance of SPHERE 5 should not change after deleting SPHERE 4.");

  WbNodeRef shape_node = wb_supervisor_node_get_from_def("SHAPE");
  WbFieldRef geometry_field = wb_supervisor_node_get_field(shape_node, "geometry");

  wb_supervisor_field_import_sf_node_from_string(geometry_field, "DEF SPHERE_GEOM Sphere { radius 0.1 }");
  const double new_translation[] = {0.15, 0.0, 0.0};
  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_parent_node(shape_node), "translation"),
                                   new_translation);
  wb_robot_step(TIME_STEP);

  value0 = wb_distance_sensor_get_value(ds0);
  ts_assert_double_is_bigger(750, value0, "Sphere not imported.");
  wb_supervisor_field_remove_sf(geometry_field);

  wb_robot_step(TIME_STEP);

  // this import should fail because field is not empty
  wb_supervisor_field_import_sf_node_from_string(geometry_field, "DEF SPHERE_GEOM Sphere { radius 15 }");
  value0 = wb_distance_sensor_get_value(ds0);
  ts_assert_double_is_bigger(value0, 750, "Import on non-empty SFNode wrongly succeeded.");
  wb_supervisor_field_remove_sf(geometry_field);

  wb_robot_step(TIME_STEP);
  value0 = wb_distance_sensor_get_value(ds0);
  ts_assert_double_is_bigger(value0, 750, "Sphere not removed.");

  // test subnodes are correctly removed from Supervisor API's internal node list
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1,
                                                 "DEF PARENT Group { children [ Shape { geometry DEF CHILD Cylinder {} } ] }");
  wb_robot_step(TIME_STEP);
  WbNodeRef parent_node = wb_supervisor_node_get_from_def("PARENT");
  ts_assert_pointer_not_null(parent_node, "Invalid reference to node 'PARENT'.");
  WbNodeRef child_node = wb_supervisor_node_get_from_def("CHILD");
  ts_assert_pointer_not_null(child_node, "Invalid reference to node 'CHILD'.");
  ts_assert_int_not_equal(wb_supervisor_node_get_id(child_node), -1, "'CHILD' is not valid after import.");
  wb_robot_step(TIME_STEP);
  wb_supervisor_node_remove(parent_node);
  wb_robot_step(TIME_STEP);
  ts_assert_int_equal(wb_supervisor_node_get_id(child_node), -1, "'CHILD' is still valid after delete.");
  child_node = wb_supervisor_node_get_from_def("CHILD");
  ts_assert_pointer_null(child_node, "Invalid reference to node 'CHILD'.");

  ts_send_success();
  return EXIT_SUCCESS;
}
