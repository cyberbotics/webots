#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define EXPECTED 970.0

int main(int argc, char **argv) {
  double value;
  ts_setup(argv[0]);

  WbNodeRef object_node = wb_supervisor_node_get_from_def("OBJECT");
  WbFieldRef color_field = wb_supervisor_node_get_field(object_node, "color");

  WbDeviceTag ds = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(ds, TIME_STEP);

  wb_robot_step(TIME_STEP);

  // 970 when radius is correct
  value = wb_distance_sensor_get_value(ds);
  ts_assert_double_in_delta(value, EXPECTED, 1.0, "Unexpected initial object size: measured %f, expected %f.", value, EXPECTED);

  double new_color[3] = {0.0, 1.0, 0.0};
  wb_supervisor_field_set_sf_color(color_field, new_color);

  wb_robot_step(TIME_STEP);

  // value = 170 if parameter value is not correctly applied
  value = wb_distance_sensor_get_value(ds);
  ts_assert_double_in_delta(value, EXPECTED, 1.0, "Unexpected object size after PROTO regeneration: measured %f, expected %f.",
                            value, EXPECTED);

  ts_send_success();
  return EXIT_SUCCESS;
}
