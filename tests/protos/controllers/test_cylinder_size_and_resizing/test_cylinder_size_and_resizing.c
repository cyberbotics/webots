#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  WbDeviceTag ds;
  WbNodeRef parameterNode, templateBoxNode;
  WbFieldRef heightField, xSizeField;
  double value;

  ts_setup(argv[1]);  // give the controller args

  ds = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(ds, TIME_STEP);

  parameterNode = wb_supervisor_node_get_from_def("PARAMETER");
  heightField = wb_supervisor_node_get_field(parameterNode, "height");

  templateBoxNode = wb_supervisor_node_get_from_def("TEMPLATE_BOX");
  xSizeField = wb_supervisor_node_get_field(templateBoxNode, "xSize");

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds);
  ts_assert_double_in_delta(value, 400, 5,
                            "Unexpected cylinder position due to incorrect parenting of derived parameter node instances.");

  // resize cylinder
  wb_supervisor_field_set_sf_float(heightField, 0.15);

  // resize box
  wb_supervisor_field_set_sf_float(xSizeField, 0.2);

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds);
  ts_assert_double_in_delta(value, 1000, 5,
                            "Unexpected cylinder size due to incorrect redirection of derived default parameter.");

  if (argc > 2 && strcmp(argv[2], "template_regeneration") == 0) {
    // check translation field redirection after node regeneration
    WbNodeRef translationNode;
    WbFieldRef translationField;
    translationNode = wb_supervisor_node_get_from_def("T");
    translationField = wb_supervisor_node_get_field(translationNode, "translation");

    // move cylinder
    double newTranslation[3] = {0.0, 0.15, 0.0};
    wb_supervisor_field_set_sf_vec3f(translationField, newTranslation);

    wb_robot_step(TIME_STEP);

    value = wb_distance_sensor_get_value(ds);
    ts_assert_double_in_delta(
      value, 400, 5, "Unexpected cylinder position due to incorrect redirection of normal fields after template regeneration.");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
