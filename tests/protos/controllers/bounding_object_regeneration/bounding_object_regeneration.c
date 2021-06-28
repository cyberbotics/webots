#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);  // give the controller args

  WbNodeRef probe_node = wb_supervisor_node_get_from_def("PROBE");
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("ROBOT");

  WbFieldRef translation_field = wb_supervisor_node_get_field(probe_node, "translation");

  wb_robot_step(TIME_STEP);

  int number_of_contact_points;
  WbContactPoint *contact_points_array = wb_supervisor_node_get_contact_points(probe_node, true, &number_of_contact_points);

  wb_robot_step(TIME_STEP);

  printf("%d\n", number_of_contact_points);

  const double position[3] = {0.5, 0, 0};
  wb_supervisor_field_set_sf_vec3f(translation_field, position);

  while (wb_robot_step(TIME_STEP) != -1) {
    contact_points_array = wb_supervisor_node_get_contact_points(probe_node, true, &number_of_contact_points);
    // number_of_contact_points = wb_supervisor_node_get_number_of_contact_points(probe_node, true);
    printf("in loop: %d\n", number_of_contact_points);
  }

  /*
  WbDeviceTag ds1 = wb_robot_get_device("ds1");
  WbDeviceTag ds2 = wb_robot_get_device("ds2");
  wb_distance_sensor_enable(ds1, TIME_STEP);
  wb_distance_sensor_enable(ds2, TIME_STEP);

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds1);
  ts_assert_double_is_bigger(value, 500, "Unexpected shaped: default base PROTO Box shape shouldn't be created.");

  value = wb_distance_sensor_get_value(ds2);
  ts_assert_double_is_bigger(1000, value, "Unexpected shaped: derived PROTO Cylinder shape should be created.");
  */
  ts_send_success();
  return EXIT_SUCCESS;
}
