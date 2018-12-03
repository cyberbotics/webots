#include <stdio.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag ts = wb_robot_get_device("touch sensor");

  wb_touch_sensor_enable(ts, TIME_STEP);

  int i;
  for (i = 0; i < 5; i++)
    wb_robot_step(TIME_STEP);

  double value = wb_touch_sensor_get_value(ts);

  ts_assert_double_in_delta(value, 0.0, 0.001, "The \"force\" TouchSensor should return 0.0 N in free fall.");

  for (i = 0; i < 100; i++)
    wb_robot_step(TIME_STEP);

  value = wb_touch_sensor_get_value(ts);

  ts_assert_double_in_delta(value, 981.0, 1.0, "The \"force\" TouchSensor doesn't return the right value when on the floor.");

  ts_send_success();
  return EXIT_SUCCESS;
}
