#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int time_step = (int)wb_robot_get_basic_time_step();

  WbDeviceTag touch_sensor = wb_robot_get_device("touch sensor");
  wb_touch_sensor_enable(touch_sensor, time_step);

  while (wb_robot_step(time_step) != -1) {
    if (wb_robot_get_time() > 2.0)
      ts_assert_boolean_equal(0, "Time-out reached before collisions are detected");

    if (wb_touch_sensor_get_value(touch_sensor) != 0.0)
      break;
  };

  ts_assert_double_equal(wb_robot_get_time(), 0.48, "Falling time not correct, %lf instead of %lf", wb_robot_get_time(), 0.48);

  ts_send_success();
  return EXIT_SUCCESS;
}
