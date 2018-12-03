#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/touch_sensor.h>

#include <stdio.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 8

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag ds_box = wb_robot_get_device("dsBox");
  wb_distance_sensor_enable(ds_box, TIME_STEP);

  WbDeviceTag ds_cylinder = wb_robot_get_device("dsCylinder");
  wb_distance_sensor_enable(ds_cylinder, TIME_STEP);

  WbDeviceTag ds_sphere = wb_robot_get_device("dsSphere");
  wb_distance_sensor_enable(ds_sphere, TIME_STEP);

  WbDeviceTag ds_capsule = wb_robot_get_device("dsCapsule");
  wb_distance_sensor_enable(ds_capsule, TIME_STEP);

  WbDeviceTag ds_ifs = wb_robot_get_device("dsIndexedFaceSet");
  wb_distance_sensor_enable(ds_ifs, TIME_STEP);

  WbDeviceTag ts_box = wb_robot_get_device("tsBox");
  wb_touch_sensor_enable(ts_box, TIME_STEP);

  WbDeviceTag ts_cylinder = wb_robot_get_device("tsCylinder");
  wb_touch_sensor_enable(ts_cylinder, TIME_STEP);

  WbDeviceTag ts_sphere = wb_robot_get_device("tsSphere");
  wb_touch_sensor_enable(ts_sphere, TIME_STEP);

  WbDeviceTag ts_capsule = wb_robot_get_device("tsCapsule");
  wb_touch_sensor_enable(ts_capsule, TIME_STEP);

  WbDeviceTag ts_ifs = wb_robot_get_device("tsIndexedFaceSet");
  wb_touch_sensor_enable(ts_ifs, TIME_STEP);

  WbNodeRef bo_box = wb_supervisor_node_get_from_def("BO_BOX");
  WbFieldRef box_size_field = wb_supervisor_node_get_field(bo_box, "size");

  WbNodeRef bo_cylinder = wb_supervisor_node_get_from_def("BO_CYLINDER");
  WbFieldRef cylinder_radius_field = wb_supervisor_node_get_field(bo_cylinder, "radius");

  WbNodeRef bo_sphere = wb_supervisor_node_get_from_def("BO_SPHERE");
  WbFieldRef sphere_radius_field = wb_supervisor_node_get_field(bo_sphere, "radius");

  WbNodeRef bo_capsule = wb_supervisor_node_get_from_def("BO_CAPSULE");
  WbFieldRef capsule_height_field = wb_supervisor_node_get_field(bo_capsule, "height");

  WbNodeRef bo_ifs_coord = wb_supervisor_node_get_from_def("BO_INDEXED_FACE_SET_COORD");
  WbFieldRef coord_point_field = wb_supervisor_node_get_field(bo_ifs_coord, "point");

  wb_robot_step(2 * TIME_STEP);

  const double box_size[3] = {1.0, 0.1, 0.1};
  wb_supervisor_field_set_sf_vec3f(box_size_field, box_size);

  wb_supervisor_field_set_sf_float(cylinder_radius_field, 0.5);

  wb_supervisor_field_set_sf_float(sphere_radius_field, 0.35);

  wb_supervisor_field_set_sf_float(capsule_height_field, 1.5);

  const double new_coord[3] = {1.0, -0.05, 0};
  wb_supervisor_field_set_mf_vec3f(coord_point_field, 2, new_coord);

  wb_robot_step(TIME_STEP);

  double ds_box_value = wb_distance_sensor_get_value(ds_box);
  ts_assert_double_in_delta(ds_box_value, 300.0, 0.1, "Box scaling not detected from the distance sensor (received = %f)",
                            ds_box_value);

  double ds_cylinder_value = wb_distance_sensor_get_value(ds_cylinder);
  ts_assert_double_in_delta(ds_cylinder_value, 300.0, 0.1,
                            "Cylinder scaling not detected from the distance sensor (received = %f)", ds_cylinder_value);

  double ds_sphere_value = wb_distance_sensor_get_value(ds_sphere);
  ts_assert_double_in_delta(ds_sphere_value, 300.0, 0.1, "Sphere scaling not detected from the distance sensor (received = %f)",
                            ds_sphere_value);

  double ds_capsule_value = wb_distance_sensor_get_value(ds_capsule);
  ts_assert_double_in_delta(ds_capsule_value, 300.0, 0.1,
                            "Capsule scaling not detected from the distance sensor (received = %f)", ds_capsule_value);

  double ds_ifs_value = wb_distance_sensor_get_value(ds_ifs);
  ts_assert_double_in_delta(ds_ifs_value, 300.0, 0.1,
                            "IndexedFaceSet scaling not detected from the distance sensor (received = %f)", ds_ifs_value);

  while (wb_robot_get_time() < 1.0) {
    wb_robot_step(TIME_STEP);

    double ts_box_value = wb_touch_sensor_get_value(ts_box);
    ts_assert_int_equal(ts_box_value, 0, "Object passed trough the scaled box");

    double ts_cylinder_value = wb_touch_sensor_get_value(ts_cylinder);
    ts_assert_int_equal(ts_cylinder_value, 0, "Object passed trough the scaled cylinder");

    double ts_sphere_value = wb_touch_sensor_get_value(ts_sphere);
    ts_assert_int_equal(ts_sphere_value, 0, "Object passed trough the scaled sphere");

    double ts_capsule_value = wb_touch_sensor_get_value(ts_capsule);
    ts_assert_int_equal(ts_capsule_value, 0, "Object passed trough the scaled capsule");

    double ts_ifs_value = wb_touch_sensor_get_value(ts_ifs);
    ts_assert_int_equal(ts_ifs_value, 0, "Object passed trough the scaled indexed face set");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
