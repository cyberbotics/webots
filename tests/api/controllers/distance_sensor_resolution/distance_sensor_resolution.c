#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <string.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag ds[5];
  double resolution[5] = {0, 0.01, 0.1, 0.3, 15};
  double value[5], reference;
  int c;

  ds[0] = wb_robot_get_device("ds0");
  ds[1] = wb_robot_get_device("ds1");
  ds[2] = wb_robot_get_device("ds2");
  ds[3] = wb_robot_get_device("ds3");
  ds[4] = wb_robot_get_device("ds4");

  for (c = 0; c < 5; ++c)
    wb_distance_sensor_enable(ds[c], TIME_STEP);

  wb_robot_step(TIME_STEP);

  // get value of each distance_sensor
  for (c = 0; c < 5; ++c)
    value[c] = wb_distance_sensor_get_value(ds[c]);

  // check ds1 to ds4 respectively to ds0
  for (c = 1; c < 5; ++c) {
    reference = ((int)(value[c] / resolution[c] + 0.5)) * resolution[c];
    ts_assert_double_equal(value[c], reference, "Distance sensor 'ds%d' doesn't respect it's resolution field", c);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
