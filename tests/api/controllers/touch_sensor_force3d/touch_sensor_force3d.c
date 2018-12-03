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

  const double *values = wb_touch_sensor_get_values(ts);

  for (i = 0; i < 3; i++)
    ts_assert_double_in_delta(values[i], 0.0, 0.0001,
                              "The \"force3d\" TouchSensor should return [0.0 0.0 0.0] N in free fall.");

  for (i = 0; i < 100; i++)
    wb_robot_step(TIME_STEP);

  values = wb_touch_sensor_get_values(ts);

  const double expected[] = {0.0, 0.0, -9.81};

  for (i = 0; i < 3; i++)
    ts_assert_double_in_delta(values[i], expected[i], 0.1,
                              "The \"force3d\" TouchSensor doesn't return the right values when on the floor.");

  ts_send_success();
  return EXIT_SUCCESS;
}
