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

  int lookup_table_size = wb_touch_sensor_get_lookup_table_size(ts);
  ts_assert_double_equal(lookup_table_size, 0, "Lookup table size returned is wrong (%d instead of 0)", lookup_table_size);
  const double *lookup_table = wb_touch_sensor_get_lookup_table(ts);
  ts_assert_pointer_null((void *)lookup_table, "Lookup table returned is wrong (%p instead of NULL)", lookup_table);

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
