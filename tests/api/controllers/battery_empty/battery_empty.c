#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  const int time_Step = wb_robot_get_basic_time_step();

  wb_robot_battery_sensor_enable(time_Step);

  wb_robot_step(time_Step);
  ts_assert_double_equal(wb_robot_battery_sensor_get_value(), -1.0,
                         "When the 'battery' field is empty 'wb_robot_battery_sensor_get_value' should return '-1.0'");

  ts_send_success();
  return EXIT_SUCCESS;
}
