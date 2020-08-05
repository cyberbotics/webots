#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  double value;
  ts_setup(argv[0]);  // give the controller args

  WbNodeRef boxNode = wb_supervisor_node_get_from_def("BOX");
  WbFieldRef widthField = wb_supervisor_node_get_field(boxNode, "width");

  WbDeviceTag ds1 = wb_robot_get_device("ds1");
  WbDeviceTag ds2 = wb_robot_get_device("ds2");
  wb_distance_sensor_enable(ds1, TIME_STEP);
  wb_distance_sensor_enable(ds2, TIME_STEP);

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds1);
  ts_assert_double_in_delta(value, 1000, 10, "Unexpected box initial width for distance sensor 'ds1'.");
  value = wb_distance_sensor_get_value(ds2);
  ts_assert_double_in_delta(value, 1000, 10, "Unexpected box initial width for distance sensor 'ds2'.");

  wb_supervisor_field_set_sf_float(widthField, 0.25);

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds1);
  ts_assert_double_is_bigger(500.0, value, "Unexpected box width after first update for distance sensor 'ds1'.");
  value = wb_distance_sensor_get_value(ds2);
  ts_assert_double_in_delta(value, 1000, 10, "Unexpected box width after first update for distance sensor 'ds2'.");

  wb_supervisor_field_set_sf_float(widthField, 0.5);

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds1);
  ts_assert_double_is_bigger(500.0, value, "Unexpected box width after second update for distance sensor 'ds1'.");
  value = wb_distance_sensor_get_value(ds2);
  ts_assert_double_is_bigger(500.0, value, "Unexpected box width after second update for distance sensor 'ds2'.");

  ts_send_success();
  return EXIT_SUCCESS;
}
