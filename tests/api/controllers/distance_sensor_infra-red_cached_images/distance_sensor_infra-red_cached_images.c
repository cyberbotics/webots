#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag ds_left = wb_robot_get_device("distance sensor left");
  WbDeviceTag ds_right = wb_robot_get_device("distance sensor right");
  wb_distance_sensor_enable(ds_left, TIME_STEP);
  wb_distance_sensor_enable(ds_right, TIME_STEP);

  wb_robot_step(TIME_STEP);

  ts_assert_double_in_delta(wb_distance_sensor_get_value(ds_left), wb_distance_sensor_get_value(ds_right), 0.1,
                            "Both of the distance sensors should return the same value regardless if they are pointing to the "
                            "original or cached texture");

  ts_send_success();
  return EXIT_SUCCESS;
}
