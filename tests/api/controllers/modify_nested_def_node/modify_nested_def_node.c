#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <stdio.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int time_step = wb_robot_get_basic_time_step();

  WbDeviceTag ds_def0 = wb_robot_get_device("ds0");  // generic
  WbDeviceTag ds_def1 = wb_robot_get_device("ds1");  // generic
  WbDeviceTag ds_use1 = wb_robot_get_device("ds2");  // generic
  WbDeviceTag ds_def2 = wb_robot_get_device("ds3");  // generic
  WbDeviceTag ds_use2 = wb_robot_get_device("ds4");  // infra-red (no bounding object)
  WbDeviceTag ds_use3 = wb_robot_get_device("ds5");  // infra-red (no bounding object)
  wb_distance_sensor_enable(ds_def0, time_step);
  wb_distance_sensor_enable(ds_def1, time_step);
  wb_distance_sensor_enable(ds_use1, time_step);
  wb_distance_sensor_enable(ds_def2, time_step);
  wb_distance_sensor_enable(ds_use2, time_step);
  wb_distance_sensor_enable(ds_use3, time_step);

  wb_robot_step(time_step);

  // check initial size

  double def0 = wb_distance_sensor_get_value(ds_def0);
  double def1 = wb_distance_sensor_get_value(ds_def1);
  double use1 = wb_distance_sensor_get_value(ds_use2);
  double def2 = wb_distance_sensor_get_value(ds_def2);
  double use2 = wb_distance_sensor_get_value(ds_use2);
  double use3 = wb_distance_sensor_get_value(ds_use3);

  // nothing detected: values = 1000.000
  ts_assert_double_in_delta(def0, 1000.0, 1.0, "The initial size of box B0 is wrong.");
  ts_assert_double_in_delta(def1, 1000.0, 1.0, "The initial size of box B1 is wrong.");
  ts_assert_double_in_delta(use1, 1000.0, 1.0, "The initial size of USE node containing box B1 is wrong.");
  ts_assert_double_in_delta(def2, 1000.0, 1.0, "The initial size of box B2 is wrong.");
  ts_assert_double_in_delta(use2, 1000.0, 1.0, "The initial size of USE node containing box B2 is wrong.");
  ts_assert_double_in_delta(use3, 1000.0, 1.0, "The initial translation of USE node in USE_CASE_4 is wrong.");

  // check size after modifying PROTO parameter
  WbNodeRef protoNode = wb_supervisor_node_get_from_def("USE_CASE_0.PROTO_SHAPE");
  WbFieldRef sizeField = wb_supervisor_node_get_field(protoNode, "size");
  const double newSize[3] = {0.3, 0.1, 0.1};
  wb_supervisor_field_set_sf_vec3f(sizeField, newSize);

  WbNodeRef boxNode = wb_supervisor_node_get_from_def("B1");
  sizeField = wb_supervisor_node_get_field(boxNode, "size");
  wb_supervisor_field_set_sf_vec3f(sizeField, newSize);

  boxNode = wb_supervisor_node_get_from_def("B2");
  sizeField = wb_supervisor_node_get_field(boxNode, "size");
  wb_supervisor_field_set_sf_vec3f(sizeField, newSize);

  wb_robot_step(time_step);

  // check size after modifying the DEF node Box size
  def0 = wb_distance_sensor_get_value(ds_def0);
  def1 = wb_distance_sensor_get_value(ds_def1);
  use1 = wb_distance_sensor_get_value(ds_use1);
  def2 = wb_distance_sensor_get_value(ds_def2);
  use2 = wb_distance_sensor_get_value(ds_use2);

  // object detected: values < 1000.000
  ts_assert_double_in_delta(def0, 500.0, 1.0, "The size of box B0 is wrong after modifying it.");
  ts_assert_double_in_delta(def1, 500.0, 1.0, "The size of box B1 is wrong after modifying it.");
  ts_assert_double_in_delta(use1, 500.0, 1.0, "The size of USE node containing box B1 after modifying it.");
  ts_assert_double_in_delta(def2, 500.0, 1.0, "The size of box B2 is wrong after modifying it.");
  ts_assert_double_in_delta(use2, 526.0, 10.0, "The size of of USE node containing box B2 is wrong after modifying it.");

  // modify P2 node translation check position of nested USE nodes

  WbNodeRef poseNode = wb_supervisor_node_get_from_def("P2");
  WbFieldRef translationField = wb_supervisor_node_get_field(poseNode, "translation");
  const double newTranslation[3] = {0.4, -0.2, 0};
  wb_supervisor_field_set_sf_vec3f(translationField, newTranslation);

  wb_robot_step(time_step);

  use3 = wb_distance_sensor_get_value(ds_use3);

  // object detected: values < 1000.000
  ts_assert_double_in_delta(use3, 504.0, 1.0, "The translation of USE node in USE_CASE_4 is wrong after modifying it.");

  ts_send_success();
  return EXIT_SUCCESS;
}
