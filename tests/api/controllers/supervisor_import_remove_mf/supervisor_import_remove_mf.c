#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  wb_robot_step(TIME_STEP);
  double reference_distance = 0.45;

  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  WbNodeRef node = wb_supervisor_node_get_from_def("NODE");
  WbFieldRef bool_field = wb_supervisor_node_get_field(node, "bool");
  WbFieldRef int_field = wb_supervisor_node_get_field(node, "int");
  WbFieldRef float_field = wb_supervisor_node_get_field(node, "float");
  WbFieldRef vec2_field = wb_supervisor_node_get_field(node, "vec2");
  WbFieldRef vec3_field = wb_supervisor_node_get_field(node, "vec3");
  WbFieldRef color_field = wb_supervisor_node_get_field(node, "color");
  WbFieldRef rotation_field = wb_supervisor_node_get_field(node, "rot");
  WbFieldRef string_field = wb_supervisor_node_get_field(node, "string");

  const double vec2_value[2] = {0.1, 0.2};
  const double vec3_value[3] = {0.1, 0.2, 0.3};
  const double rotation_value[4] = {0.0, 1.0, 0.0, 0.57};
  const double color_value[3] = {0.1, 0.2, 0.3};

  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Initial distance incorrect.");

  // Import

  wb_robot_step(TIME_STEP);

  wb_supervisor_field_insert_mf_bool(bool_field, 0, true);
  reference_distance -= 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after insertion of MFBool incorrect.");

  wb_supervisor_field_insert_mf_int32(int_field, -1, 17);
  reference_distance -= 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after insertion of MFInt32 incorrect.");

  wb_supervisor_field_insert_mf_float(float_field, 0, 0.54);
  reference_distance -= 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after insertion of MFFloat incorrect.");

  wb_supervisor_field_insert_mf_vec2f(vec2_field, -1, vec2_value);
  reference_distance -= 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after insertion of MFVec2f incorrect.");

  wb_supervisor_field_insert_mf_vec3f(vec3_field, 0, vec3_value);
  reference_distance -= 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after insertion of MFVec3f incorrect.");

  wb_supervisor_field_insert_mf_rotation(rotation_field, -1, rotation_value);
  reference_distance -= 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after insertion of MFRotation incorrect.");

  wb_supervisor_field_insert_mf_color(color_field, 0, color_value);
  reference_distance -= 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after insertion of MFColor incorrect.");

  wb_supervisor_field_insert_mf_string(string_field, -1, "Hello World!");
  const char *expected_string = "Second string";
  wb_supervisor_field_insert_mf_string(string_field, 1, expected_string);
  const char *second_string = wb_supervisor_field_get_mf_string(string_field, 1);
  ts_assert_string_equal(second_string, expected_string, "Order of items inserted in MFString is incorrect.");
  reference_distance -= 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after insertion of MFString incorrect.");

  // Remove

  wb_supervisor_field_remove_mf(string_field, 0);
  reference_distance += 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after removing MFString incorrect.");

  wb_supervisor_field_remove_mf(color_field, -1);
  reference_distance += 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after removing MFColor incorrect.");

  wb_supervisor_field_remove_mf(rotation_field, -1);
  reference_distance += 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after removing MFRotation incorrect.");

  wb_supervisor_field_remove_mf(vec3_field, 0);
  reference_distance += 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after removing MFVec3f incorrect.");

  wb_supervisor_field_remove_mf(vec2_field, 0);
  reference_distance += 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after removing MFVec2f incorrect.");

  wb_supervisor_field_remove_mf(float_field, -1);
  reference_distance += 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after removing MFFloat incorrect.");

  wb_supervisor_field_remove_mf(int_field, 0);
  reference_distance += 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after removing MFInt32 incorrect.");

  wb_supervisor_field_remove_mf(bool_field, -1);
  reference_distance += 0.05;
  wb_robot_step(TIME_STEP);
  ts_assert_double_in_delta(wb_distance_sensor_get_value(distance_sensor), reference_distance, 0.0001,
                            "Distance after removing MFBool incorrect.");

  ts_send_success();
  return EXIT_SUCCESS;
}
