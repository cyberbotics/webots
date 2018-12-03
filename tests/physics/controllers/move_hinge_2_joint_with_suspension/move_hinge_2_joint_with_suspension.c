#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int time_step = (int)wb_robot_get_basic_time_step();

  wb_robot_step(10 * time_step);

  // move robot
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("ROBOT");
  WbFieldRef translation_field = wb_supervisor_node_get_field(robot_node, "translation");
  double position[3] = {0.0, 1.01, 0.0};
  wb_supervisor_field_set_sf_vec3f(translation_field, position);

  wb_robot_step(5 * time_step);

  WbDeviceTag left_sensor = wb_robot_get_device("left distance sensor");
  WbDeviceTag right_sensor = wb_robot_get_device("right distance sensor");
  wb_distance_sensor_enable(left_sensor, time_step);
  wb_distance_sensor_enable(right_sensor, time_step);

  int steps = 50;
  while (steps > 0 && wb_robot_step(time_step) != -1) {
    ts_assert_double_is_bigger(wb_distance_sensor_get_value(left_sensor), 999.0, "Cylinder deviated to the left.");
    ts_assert_double_is_bigger(wb_distance_sensor_get_value(right_sensor), 999.0, "Cylinder deviated to the right.");
    --steps;
  }

  ts_assert_int_equal(steps, 0, "Simulation terminated before completing all the tests.");

  ts_send_success();
  return EXIT_SUCCESS;
}
