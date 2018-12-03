#include <webots/supervisor.h>
#include <webots/touch_sensor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

// 1. Pin the robot a while using the supervisor "set robot translation" API, and
// check it doesn't touch anything in this case. i.e.:
// - the physics is well reset.
// - there is no joint explosion.
// 2. Release the robot
// - check it is stable, i.e. it touches only the ground.

int main(int argc, char **argv) {
  int i;

  ts_setup(argv[0]);

  int time_step = wb_robot_get_basic_time_step();

  WbNodeRef robot_node = wb_supervisor_node_get_from_def("ROBOT");
  WbFieldRef translation_field = wb_supervisor_node_get_field(robot_node, "translation");

  WbDeviceTag ground_sensor = wb_robot_get_device("ground collider");
  wb_touch_sensor_enable(ground_sensor, time_step);
  WbDeviceTag walls_sensor = wb_robot_get_device("walls collider");
  wb_touch_sensor_enable(walls_sensor, time_step);

  double robot_target_translation[] = {0.0, 0.26, 0.0};

  // Pin the robot and check it doesn't touch anything.
  for (i = 0; i < 100; ++i) {
    wb_supervisor_field_set_sf_vec3f(translation_field, robot_target_translation);
    wb_supervisor_node_reset_physics(robot_node);
    wb_robot_step(time_step);
    ts_assert_double_equal(wb_touch_sensor_get_value(ground_sensor), 0.0, "Robot touches the ground when pinned.");
    ts_assert_double_equal(wb_touch_sensor_get_value(walls_sensor), 0.0, "Robot touches the walls when pinned.");
  }

  // Unpin the robot and check it doesn't touch the walls.
  for (i = 0; i < 100; ++i) {
    wb_robot_step(time_step);
    ts_assert_double_equal(wb_touch_sensor_get_value(walls_sensor), 0.0, "Robot touches the walls when dropped.");
  }

  // Check the robot is stable.
  for (i = 0; i < 100; ++i) {
    wb_robot_step(time_step);
    ts_assert_double_equal(wb_touch_sensor_get_value(walls_sensor), 0.0, "Robot touches the walls when it should be stable.");
    ts_assert_double_equal(wb_touch_sensor_get_value(ground_sensor), 1.0,
                           "Robot doesn't touch the ground when it should be stable.");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
