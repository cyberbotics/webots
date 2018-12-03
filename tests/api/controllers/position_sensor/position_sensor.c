#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag position_sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  wb_robot_step(TIME_STEP);

  double value = wb_position_sensor_get_value(position_sensor);
  ts_assert_double_equal(
    value, 0.0, "wb_position_sensor_get_value() did not return the correct initial position: returned %lf instead of 0.0.",
    value);

  wb_robot_step(10 * TIME_STEP);

  value = wb_position_sensor_get_value(position_sensor);
  ts_assert_double_equal(
    value, -0.09, "wb_position_sensor_get_value() did not return the correct final position: returned %lf instead of -0.09.",
    value);

  ts_send_success();
  return EXIT_SUCCESS;
}
